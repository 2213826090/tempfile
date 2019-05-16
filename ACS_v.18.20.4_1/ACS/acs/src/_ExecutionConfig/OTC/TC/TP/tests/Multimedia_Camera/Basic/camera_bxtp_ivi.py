# coding: utf-8
import os
import re
import time

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_switch_camera_helper import MultiMediaSwitchCameraHelper
from testlib.multimedia.multimedia_logcat_helper import MultiMediaLogcatHelper
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.AOSPCamera import AOSPCamera
from testlib.camera.RefCamCamera import RefCamCamera

from testlib.multimedia.multimedia_mplayer_helper import MultiMediaMplayerHelper
from testlib.multimedia.multimedia_checkiq_helper import MultiMediaCheckiqHelper

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)

        g_common_obj.root_on_device()
        self.camera_common = CameraCommon()

        self.host_path = self.camera_common.getTmpDir()

        self.rvc_camera = ""
        self.aosp_camera = ""
        self.ref_camera = ""
        self.case_result = -1

        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.get_rvc_camera_class().stopCameraApp()
        self.camera_common.unlockScreen()
        self.get_rvc_camera_class().skipAccountLoginLyout()
        self.get_rvc_camera_class().backHome()
        self.d = g_common_obj.get_device()
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()

        if self.rvc_camera != "":
            self.rvc_camera.stopCameraApp()
        if self.aosp_camera != "":
            self.aosp_camera.cleanMediaFiles()
        self.camera_common.removeDeivceFile()
        if self.case_result == 0:
            self.reboot_device()
#         self.camera_common.removeFile(self.host_path + "/*")
        time.sleep(3)
        self.get_rvc_camera_class().backHome()

    def appPrepare(self,):
        self.logger.debug("app prepare successfully")

    def check_file_corrupt(self, mediaFileCount=1):
        return self.camera_common.checkFileCorrupt(mediaFileCount)

    def get_rvc_camera_class(self):
        if self.rvc_camera == "":
            self.rvc_camera = MultiMediaSwitchCameraHelper().camera
        return self.rvc_camera

    def get_aosp_camera_class(self):
        if self.aosp_camera == "":
            self.aosp_camera = AOSPCamera()
        return self.aosp_camera

    def get_ref_camera_class(self):
        if self.ref_camera == "":
            self.ref_camera = RefCamCamera()
        return self.ref_camera

    def reboot_device(self):
        self.get_rvc_camera_class().reboot_device()

    def check_home_or_lock_layout(self, check_exist=True):
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        self.get_rvc_camera_class().skipAccountLoginLyout()
        assert self.get_rvc_camera_class().isHomeLayoutExists() or self.get_rvc_camera_class().isLockLayoutExists(), "Home or Lock layout not exist!"

    def start_stop_camera_test(self, wait_time=5, check_success=1):
        self.logger.debug("start_stop_camera_test start")
        self.get_rvc_camera_class().startCameraApp(check_success)
        time.sleep(wait_time)
        self.get_rvc_camera_class().stopCameraApp()
        time.sleep(2)

    def check_gps(self):
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.multimedia_setting.install_apk("gps_apk")
        self.camera_common.launchGpsApp()
        network_longitude, network_latitude = self.camera_common.getGPSLocation()
        return network_longitude, network_latitude

    def check_logcat_data(self, data_list, expect_lower_limit_fps, expect_upper_limit_fps):
        self.logger.debug("check_logcat_data data_list=%s" % data_list)
        if len(data_list) == 0:
            return False, -1
        for t_fps in data_list:
            t_fps = float(t_fps)
            if expect_lower_limit_fps > t_fps or t_fps > expect_upper_limit_fps:
                return False, t_fps
        return True, 1

    def check_aosp_camera_exif_info(self, check_info_dict):
        self.case_result = 0
        self.get_aosp_camera_class().startCameraApp()
        self.get_aosp_camera_class().selectMode("Camera")
        self.get_aosp_camera_class().capturePhoto(1)
        file_info, file_name = self.check_file_corrupt()
        for check_info_key in check_info_dict.keys():
            assert check_info_key in file_info.keys(), "\"%s\" not in exif info!" % check_info_key
            actual_info = file_info[check_info_key]
            expect_info_parttern = re.compile(check_info_dict[check_info_key])
            self.logger.debug("actual_info = \"%s\", expect_info_parttern = \"%s\"" % (actual_info, check_info_dict[check_info_key]))
            result_list = expect_info_parttern.findall(actual_info)
            self.logger.debug(result_list)
            assert result_list != [], "actual_info = \"%s\", expect_info_parttern = \"%s\", result_list = \"%s\"" % (actual_info, check_info_dict[check_info_key], result_list)
        self.logger.debug("info:%s" % file_info)
        self.get_aosp_camera_class().stopCameraApp()
        self.get_aosp_camera_class().cleanMediaFiles()
        self.case_result = 1

    def check_ref_camera_preview_with_format(self, preview_format, duration=10):
        self.android_version = MultiMediaSwitchCameraHelper(skip_import_camera=True).android_version.lower()
        if "o" in self.android_version:
            self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
            self.multimedia_setting.install_apk("for_o_image_camera_preview_test_apk")
            from testlib.multimedia.get_device_port_helper import GetConfigFileHelper
            self.scale_test_cfg = GetConfigFileHelper("", "multimedia_scale_test_helper.conf").get_section("config")
            self.o_image_play_cmd = self.scale_test_cfg.get("o_image_play_cmd")
            check_size_list = ["720x480", "640x480", "320x240", "176x144"]
            from testlib.graphics.common import adb32
            adb32._adb_reboot()
            time.sleep(15)
            for size in check_size_list:
                width, height = size.split("x")
                result = g_common_obj.adb_cmd_capture_msg(self.o_image_play_cmd % (width, height, preview_format, duration))
                assert "OK" in result, "play error! result=%s" % result
                time.sleep(2)
        else:
            self.get_ref_camera_class().startCameraApp()
            self.get_ref_camera_class().selectMode("SingleShot")
            self.get_ref_camera_class().setSettingsButton("Preview Format", preview_format)
            check_size_list = ["1920x1080", "1280x720", "720x480", "640x480", "320x240", "176x144"]
            for size in check_size_list:
                self.logger.debug("capture photo, format: %s, size: %s" % (preview_format, size))
                self.get_ref_camera_class().setSettingsButton("Picture Size", size)
                self.get_ref_camera_class().capturePhoto()
            self.camera_common.checkFileCorrupt(len(check_size_list))

    def test_Camera_Launch_With_Press_Power(self):
        self.get_rvc_camera_class.reboot_device()
        self.logger.debug("---start sleep---")
#         time.sleep(3)
#         self.logger.debug("---start pressPowerKey---")
#         self.camera.pressPowerKey(2)
#         self.wait_boot_completed()
        time.sleep(5)
        self.start_stop_camera_test(10)
        time.sleep(10)
        self.start_stop_camera_test()

    def test_Camera_IVI_RVC_Preview_Horizontal(self):
        self.get_rvc_camera_class().startCameraApp()
        time.sleep(5)
        d = g_common_obj.get_device()
        x = d.info["displayWidth"]
        y = d.info["displayHeight"]
        self.logger.debug("display size--- x:%s, y:%s" % (str(x), str(y)))
        assert int(x) > int(y), "RVC FOV not display as horizontal"
        self.get_rvc_camera_class().stopCameraApp()

    def test_Camera_IVI_RVC_ColdBoot_Available_10Mins(self):######
        self.reboot_device()
        self.get_rvc_camera_class().startCameraApp()
        self.logger.debug("Wait 10mins...")
        time.sleep(600)
        self.get_rvc_camera_class().startCameraApp()
        self.get_rvc_camera_class().stopCameraApp()
        time.sleep(5)
        self.check_home_or_lock_layout()

    def test_Camera_IVI_RVC_ColdBoot_off_Available(self):######
        self.reboot_device()
        for _ in range(5):
            self.start_stop_camera_test()
        self.check_home_or_lock_layout()

    def test_Camera_IVI_RVC_ColdBoot_Switch_Repeatly(self):######
        self.reboot_device()
        for _ in range(5):
            self.start_stop_camera_test()
        self.check_home_or_lock_layout()

    def test_Camera_IVI_RVC_ColdBoot_Repeatly(self):######
        for i in range(10):
            self.logger.debug("------loop %s------" % str(i+1))
            self.reboot_device()
            self.get_rvc_camera_class().stopCameraApp()
            time.sleep(5)
            self.get_rvc_camera_class().startCameraApp()
        time.sleep(5)
        self.get_rvc_camera_class().stopCameraApp()

    def test_Camera_IVI_RVC_ColdBoot_on_Available(self):
        self.reboot_device()
        self.get_rvc_camera_class().backHome()
        self.check_home_or_lock_layout()
        self.start_stop_camera_test()

    def test_Camera_IVI_RVC_RunTime_Available(self):
        self.reboot_device()
        self.get_rvc_camera_class().startCameraApp()
        time.sleep(5)
        self.get_rvc_camera_class().stopCameraApp()
        time.sleep(5)
        self.check_home_or_lock_layout()

    def test_Camera_IVI_RVC_RunTime_Available_10Mins(self):
        self.reboot_device()
        self.get_rvc_camera_class().startCameraApp()
        self.logger.debug("Wait 10mins...")
        time.sleep(600)
        self.get_rvc_camera_class().stopCameraApp()
        time.sleep(5)
        self.check_home_or_lock_layout()
        self.get_rvc_camera_class().startCameraApp()
        self.get_rvc_camera_class().stopCameraApp()
        time.sleep(5)
        self.check_home_or_lock_layout()

    def test_Camera_IVI_RVC_RunTime_On_Quickly(self):
        self.reboot_device()
        for i in range(1,6):
            self.logger.debug("------loop %s------" % i)
            self.get_rvc_camera_class().startCameraApp()
            self.get_rvc_camera_class().stopCameraApp()
            time.sleep(3)

    def test_Camera_IVI_RVC_Switch_From_CameraAPP(self):
        self.check_home_or_lock_layout()
        self.get_aosp_camera_class().startCameraApp()
        for i in range(1,6):
            self.logger.debug("------loop %s------" % i)
            self.get_rvc_camera_class().startCameraApp()
            self.get_rvc_camera_class().stopCameraApp()
            self.get_aosp_camera_class().checkCameraApp()
            time.sleep(3)

    def test_Camera_IVI_AOSPCamera_Launch_Repeat(self):
        self.check_home_or_lock_layout()
        count = 20
        for i in range(1, count + 1):
            self.logger.debug("------loop %s------" % i)
            self.get_aosp_camera_class().startCameraApp()
            self.get_aosp_camera_class().checkCameraApp()
            self.get_aosp_camera_class().stopCameraApp()
            time.sleep(3)

    def test_Camera_IVI_AOSP_Rear_Launch(self):
        self.check_home_or_lock_layout()
        self.get_aosp_camera_class().startCameraApp()
        self.get_aosp_camera_class().checkCameraApp()
        self.get_aosp_camera_class().stopCameraApp()

    def test_Camera_IVI_Rear_Preview_CVBS1(self):
        self.test_Camera_IVI_AOSP_Rear_Launch()

    def test_Camera_FIT_RVC_Lock_With_PW(self):
        try:
#             self.multimedia_setting.install_apk("unlock_app")
            self.camera_common.setLockScreenWithPasswd()
            time.sleep(1)
            self.camera_common.pressPower()
            time.sleep(2)
            d = g_common_obj.get_device()
            d.wakeup()
            time.sleep(2)
            for i in range(1,6):
                self.logger.debug("------loop %s------" % i)
                self.get_rvc_camera_class().startCameraApp()
                time.sleep(5)
                self.get_rvc_camera_class().stopCameraApp()
                time.sleep(3)
            self.camera_common.unlockScreen()
#             self.camera_common.launchUnlockAppToUnlockScreen()
        except Exception as e:
            self.assertTrue(False, e)
        finally:
            self.get_rvc_camera_class().stopCameraApp()
            self.camera_common.setLockScreenWithPasswd(False)

    def test_Camera_IVI_AOSP_Metadata_EXIF_CameraModelName(self):
        self.check_aosp_camera_exif_info({"Camera Model Name" : "AOSP on Intel Platform"})

    def test_Camera_IVI_AOSP_Metadata_EXIF_DateTime(self):
        self.check_aosp_camera_exif_info({"Create Date" : "\d{4}:\d{2}:\d{2} \d{2}:\d{2}:\d{2}"})

    def test_Camera_IVI_AOSP_Metadata_EXIF_GPSLocation(self):
        self.check_gps()
        self.check_aosp_camera_exif_info({"GPS Position" : ".*N,.*E"})

    def test_Camera_IVI_AOSP_Metadata_EXIF_ImageDescription(self):
        self.check_aosp_camera_exif_info({"Image Description" : "Jpeg"})

    def test_Camera_IVI_AOSP_Metadata_EXIF_Make(self):
        self.check_aosp_camera_exif_info({"Make" : "Intel"})

    def test_Camera_IVI_AOSP_Metadata_EXIF_Resolution(self):
        self.check_aosp_camera_exif_info({"Image Size" : "\d{3,4}x\d{3,4}"})

    def test_Camera_Rear_PreviewFormat_NV21(self):
        self.check_ref_camera_preview_with_format("NV21")

    def test_Camera_Rear_PreviewFormat_YV12(self):
        self.check_ref_camera_preview_with_format("YV12")

    def test_Camera_IVI_Preview_PictureFormat_NTSC(self):
        self.check_ref_camera_preview_with_format("NV21")
        self.check_ref_camera_preview_with_format("YV12")

    def test_Camera_Rear_PreviewFormat_RGB565(self):
        self.check_ref_camera_preview_with_format("RGB_565")

    def test_Camera_Rear_PreviewFormat_YUY2(self):
        self.check_ref_camera_preview_with_format("YUY2")

    def test_Camera_IVI_AVM737_Preview_FPS_Range(self):
        self.multimedia_logcat_helper = MultiMediaLogcatHelper("adb logcat CameraHardwareSoc:D *:S")
        self.get_ref_camera_class().startCameraApp()
        self.get_ref_camera_class().selectMode("SingleShot")
        self.get_ref_camera_class().setSettingsButton("Preview FPS Range", "30000-30000")
        self.multimedia_logcat_helper.get_logcat_data_start()
        time.sleep(5)
        result_list = self.multimedia_logcat_helper.get_logcat_data_end("total fps is (.*),")
        check_logcat_data_result, error_fps = self.check_logcat_data(result_list, 29, 31)
        assert check_logcat_data_result, "fps error! error_fps=%s, expect_fps=%s, result_list=%s" % (error_fps, "30000-30000", result_list)
        self.get_ref_camera_class().capturePhoto()
        self.get_ref_camera_class().setSettingsButton("Preview FPS Range", "30000-60000")
        self.multimedia_logcat_helper.get_logcat_data_start()
        time.sleep(5)
        result_list = self.multimedia_logcat_helper.get_logcat_data_end("total fps is (.*),")
        check_logcat_data_result, error_fps = self.check_logcat_data(result_list, 30, 60)
        assert check_logcat_data_result, "fps error! error_fps=%s, expect_fps=%s, result_list=%s" % (error_fps, "30000-60000", result_list)
        self.get_ref_camera_class().capturePhoto()
        self.camera_common.checkFileCorrupt(2)

    def test_ref_camera_with_different_format(self, preview_format):
        assert 0, "Need OV10635 senser!"
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.multimedia_setting.executeCommandWithPopen("setprop camera.hal.perf 3")
        self.get_ref_camera_class().startCameraApp()
        self.get_ref_camera_class().selectMode("SingleShot")
        self.get_ref_camera_class().setSettingsButton("Preview Format", preview_format)
        preview_size_list = ["1280x800", "1280x720", "640x480"]
        for preview_size in preview_size_list:
            self.get_ref_camera_class().setSettingsButton("Picture Size", preview_size)
            self.get_ref_camera_class().capturePhoto()
        self.camera_common.checkFileCorrupt(len(preview_size_list))

    def test_Camera_IVI_OV10635_Preview_NV21_Available(self):
        self.test_ref_camera_with_different_format("NV21")

    def test_Camera_IVI_OV10635_Preview_RGB565_Available(self):
        self.test_ref_camera_with_different_format("RGB565")

    def test_Camera_IVI_OV10635_Preview_YUY2_Available(self):
        self.test_ref_camera_with_different_format("YUY2")

    def test_Camera_IVI_OV10635_Preview_YV12_Available(self):
        self.test_ref_camera_with_different_format("YV12")

    def test_ref_camera_check_fps(self, preview_format):
        assert 0, "Need OV10635 senser!"
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.multimedia_setting.executeCommandWithPopen("setprop camera.hal.perf 3")
        self.multimedia_logcat_helper = MultiMediaLogcatHelper("adb logcat CameraHardwareSoc:D *:S")
        self.get_ref_camera_class().startCameraApp()
        self.get_ref_camera_class().selectMode("SingleShot")
        self.get_ref_camera_class().setSettingsButton("Preview Format", preview_format)
        self.multimedia_logcat_helper.get_logcat_data_start()
        time.sleep(1*60)
        preview_size_list = ["1280x800", "1280x720", "640x480"]
        for preview_size in preview_size_list:
            self.get_ref_camera_class().setSettingsButton("Picture Size", preview_size)
            self.get_ref_camera_class().capturePhoto()
        self.camera_common.checkFileCorrupt(len(preview_size_list))
        result_list = self.multimedia_logcat_helper.get_logcat_data_end("total fps is (.*),")
        check_logcat_data_result, error_fps = self.check_logcat_data(result_list, 29, 31)
        assert check_logcat_data_result, "fps error! error_fps=%s, expect_fps=%s, result_list=%s" % (error_fps, "29-31", result_list)

    def test_Camera_IVI_OV10635_Preview_NV21_30fps(self):
        self.test_ref_camera_check_fps("NV21")

    def test_Camera_IVI_OV10635_Preview_RGB565_30fps(self):
        self.test_ref_camera_check_fps("RGB565")

    def test_Camera_IVI_OV10635_Preview_YUY2_30fps(self):
        self.test_ref_camera_check_fps("YUY2")

    def test_Camera_IVI_OV10635_Preview_YV12_30fps(self):
        self.test_ref_camera_check_fps("YV12")

    def test_Camera_IVI_OV10635_AOSPCamera_Launch_Repeat(self):
        assert 0, "Need OV10635 senser!"
        self.check_home_or_lock_layout()
        count = 20
        for i in range(1, count + 1):
            self.logger.debug("------loop %s------" % i)
            self.get_aosp_camera_class().startCameraApp()
            self.get_aosp_camera_class().checkCameraApp()
            self.get_aosp_camera_class().stopCameraApp()
            time.sleep(3)

    def test_Camera_IVI_S3_WakeUp_AOSPCamera(self):
        self.get_rvc_camera_class().pressPowerKey(2)
        wait_time = 5 * 60
        self.logger.debug("------Wait %s s------" % str(wait_time))
        time.sleep(wait_time)
        self.get_rvc_camera_class().pressPowerKey(2)
        time.sleep(10)
        self.get_aosp_camera_class().startCameraApp()
        self.get_aosp_camera_class().stopCameraApp()

    def test_Camera_IVI_AOSP_Camera_S3_Available(self):
        self.get_aosp_camera_class().startCameraApp()
        self.get_rvc_camera_class().pressPowerKey(2)
        wait_time = 5 * 60
        self.logger.debug("------Wait %s s------" % str(wait_time))
        time.sleep(wait_time)
        self.get_rvc_camera_class().pressPowerKey(2)
        time.sleep(10)
        self.get_aosp_camera_class().checkCameraApp()
        self.get_aosp_camera_class().stopCameraApp()

    def test_Camera_IVI_AOSP_S3_WakeUp_With_DifferentTime_Interval(self):
        wait_time_list = [5*60 ,20 ,30, 5*60, 10*60]
        for wait_time in wait_time_list:
            self.get_aosp_camera_class().startCameraApp()
            self.get_rvc_camera_class().pressPowerKey(2)
            self.logger.debug("------Wait %s s------" % str(wait_time))
            time.sleep(wait_time)
            self.get_rvc_camera_class().pressPowerKey(2)
            time.sleep(10)
            self.get_aosp_camera_class().checkCameraApp()
            self.get_aosp_camera_class().stopCameraApp()

    def test_Camera_IVI_RunTime_RVC_S3_Available(self):
        self.get_rvc_camera_class().startCameraApp()
        self.get_rvc_camera_class().pressPowerKey(2)
        wait_time = 5 * 60
        self.logger.debug("------Wait %s s------" % str(wait_time))
        time.sleep(wait_time)
        self.get_rvc_camera_class().pressPowerKey(2)
        time.sleep(10)
        self.get_rvc_camera_class().stopCameraApp()

    def test_Camera_IVI_S3_WakeUp_RVC(self):
        self.get_rvc_camera_class().pressPowerKey(2)
        wait_time = 5 * 60
        self.logger.debug("------Wait %s s------" % str(wait_time))
        time.sleep(wait_time)
        self.get_rvc_camera_class().pressPowerKey(2)
        time.sleep(10)
        self.get_rvc_camera_class().startCameraApp()
        self.get_rvc_camera_class().stopCameraApp()

    def test0(self):
        self.reboot_device()

    def test1(self):
        self.multimedia_mplayer_helper = MultiMediaMplayerHelper()
        self.multimedia_checkiq_helper = MultiMediaCheckiqHelper(self.host_path)

        self.multimedia_mplayer_helper.play_video("/home/auto1/tmp/video_20s.avi")
        time.sleep(2)
        self.multimedia_mplayer_helper.control_video(["pause", "pausing seek 0 2", "frame_step"])
        self.get_aosp_camera_class().startCameraApp()
        self.get_aosp_camera_class().selectMode("Video")
        self.multimedia_mplayer_helper.control_video(["pause"])
        self.get_aosp_camera_class().recordVideo(1, 5)
        self.multimedia_mplayer_helper.close_video()
        file_info, file_name = self.check_file_corrupt()
        self.multimedia_checkiq_helper.check_video_with_barcode(file_name)

    def test2(self):
        from testlib.multimedia.multimedia_canbox_helper import MultiMediaCanboxHelper
        self.multimedia_canbox_helper = MultiMediaCanboxHelper()
        self.multimedia_canbox_helper.candump_start()
        time.sleep(3)
        self.multimedia_canbox_helper.cansend("00A#01")
        time.sleep(5)
        self.multimedia_canbox_helper.cansend("00A#00")
        time.sleep(3)
        t_str = self.multimedia_canbox_helper.candump_end()
        self.logger.debug("str=%s" % t_str)

    def test3(self):
        from testlib.graphics.common import adb32
        adb32.adb_disable_verity()