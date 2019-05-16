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

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """

    MULTI_CAMERA_CONFIG_DICT = {"input_field" : 1
                                ,"input_resolution" :3
                                ,"input_format" : 4
                                ,"input_default" : 5
                                ,"output_resolution" : 0
                                ,"output_format" : 2}

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

        self.rvc_camera = MultiMediaSwitchCameraHelper().camera

        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.camera_common.unlockScreen()
        self.rvc_camera.skipAccountLoginLyout()
        self.rvc_camera.backHome()
        self.d = g_common_obj.get_device()
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()

        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        need_install_result = self.multimedia_setting.install_apk("multi_camera_apk")
        if need_install_result:
            self.reboot_device()
        self.multi_camera_package_name, _ = self.multimedia_setting.get_package_and_activity_name("multi_camera_apk")
        MULTI_CAMERA_PERMISSION_LIST = ["android.permission.CAMERA"]
        self.camera_common.grantPermission(self.multi_camera_package_name, MULTI_CAMERA_PERMISSION_LIST)

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()

        self.camera_common.removeDeivceFile()
        self.camera_common.clean_app_data(self.multi_camera_package_name)
        time.sleep(3)
        self.rvc_camera.backHome()

    def reboot_device(self):
        self.rvc_camera.pressPowerKey(10)
        time.sleep(5)
        self.rvc_camera.pressPowerKey(2)
        time.sleep(30)
        g_common_obj.root_on_device()
        self.camera_common.unlockScreen()
        self.rvc_camera.backHome()
        self.check_home_or_lock_layout()

    def set_multicamera_input_config_format(self, input_config_format):
        if "multicamera_input_config_format" in os.environ:
            previous_input_config_format = os.environ["multicamera_input_config_format"]
            if previous_input_config_format == input_config_format:
                self.logger.debug("skip set multicamera_input_config_format")
                return False
        g_common_obj.adb_cmd("setprop camera.input.config.format %s" % input_config_format)
        os.environ["multicamera_input_config_format"] = input_config_format
        self.logger.debug("set multicamera_input_config_format = %s" % input_config_format)
        return True

    def launch_multi_camera_apk(self):
        return self.multimedia_setting.launch_apk("multi_camera_apk")

    def stop_multi_camera_apk(self):
        return self.multimedia_setting.stop_apk("multi_camera_apk")

    def check_home_or_lock_layout(self, check_exist=True):
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        assert self.rvc_camera.isHomeLayoutExists() or self.rvc_camera.isLockLayoutExists(), "Home or Lock layout not exist!"

    def check_logcat_data(self, data_list, fps):
        self.logger.debug("check_logcat_data data_list=%s" % data_list)
        for t_fps in data_list:
            t_fps = float(t_fps)
            if fps * 0.97 > t_fps or t_fps > fps * 1.03:
                return False, t_fps
        return True, 1

    def set_multi_camera_config(self, config_list):
        self.logger.debug("set_multi_camera_config start. config_list=%s" % config_list)
        for config_str in config_list:
            config_item, config_value = config_str.split(":")
            self.logger.debug("config_item:%s, config_value:%s" % (config_item, config_value))
            self.d(className="android.widget.Spinner")[self.MULTI_CAMERA_CONFIG_DICT[config_item]].click.wait()
            time.sleep(1)
            self.d(textContains=config_value).click.wait()
            time.sleep(1)
        self.logger.debug("set_multi_camera_config end.")

    def check_multi_camera(self, input_size, output_size):
        config_list = ["input_field:interlaced"
                       ,"input_resolution:%s" % input_size
                       ,"input_format:default"
                       ,"input_default:HW_Weaving"
                       ,"output_resolution:%s" % output_size
                       ,"output_format:NV21"
                       ]
        self.set_multicamera_input_config_format("uyvy")
        self.launch_multi_camera_apk()
        self.set_multi_camera_config(config_list)
        self.d(textContains="Camera0:OFF").click.wait()
        time.sleep(10)
        self.camera_common.checkCameraCrash()
        assert self.d(packageName="com.example.tim.multicamera").exists, "Can't find multicamera in layout, maybe apk crashed."
        self.stop_multi_camera_apk()

    def check_multi_camera_with_fps(self, input_size, output_size):
        config_list = ["input_field:interlaced"
                       ,"input_resolution:%s" % input_size
                       ,"input_format:default"
                       ,"input_default:HW_Weaving"
                       ,"output_resolution:%s" % output_size
                       ,"output_format:NV21"
                       ]
        expect_fps = 50
        self.set_multicamera_input_config_format("uyvy")
        self.launch_multi_camera_apk()
        self.set_multi_camera_config(config_list)
        g_common_obj.adb_cmd("setprop camera.hal.perf 3")
        time.sleep(2)
        self.d(textContains="Camera0:OFF").click.wait()
        self.camera_common.checkCameraCrash()
        self.multimedia_logcat_helper = MultiMediaLogcatHelper("adb logcat CameraHardwareSoc:D *:S")
        self.multimedia_logcat_helper.get_logcat_data_start()

        time.sleep(5*60)

        result_list = self.multimedia_logcat_helper.get_logcat_data_end("total fps is (.*),")
        check_logcat_data_result, error_fps = self.check_logcat_data(result_list, expect_fps)
        assert check_logcat_data_result, "Fps error! error_fps=%s, expect_fps=%s, result_list=%s" % (error_fps, expect_fps, result_list)
        self.camera_common.checkCameraCrash()
        self.stop_multi_camera_apk()

    def test_Camera_Scale_AVM737_720x480_to_1280x720(self):
        self.check_multi_camera("720x480", "1280x720")

    def test_Camera_Scale_AVM737_720x480_to_1920x1080(self):
        self.check_multi_camera("720x480", "1920x1080")

    def test_Camera_Scale_AVM737_720x480_to_640x480(self):
        self.check_multi_camera("720x480", "640x480")

    def test_Camera_Scale_AVM737_720x480_to_720x480(self):
        self.check_multi_camera("720x480", "720x480")

    def test_Camera_Scale_AVM737_720x480_to_320x240(self):
        self.check_multi_camera("720x480", "320x240")

    def test_Camera_Scale_AVM737_720x480_to_176x144(self):
        self.check_multi_camera("720x480", "176x144")

    def test_Camera_Scale_AVM737_720x480_to_1280x720_50fps(self):
        self.check_multi_camera_with_fps("720x480", "1280x720")

    def test_Camera_Scale_AVM737_720x480_to_1920x1080_50fps(self):
        self.check_multi_camera_with_fps("720x480", "1920x1080")

    def test_Camera_Scale_AVM737_720x480_to_640x480_50fps(self):
        self.check_multi_camera_with_fps("720x480", "640x480")

    def test_Camera_Scale_AVM737_720x480_to_720x480_50fps(self):
        self.check_multi_camera_with_fps("720x480", "720x480")

    def test_Camera_Scale_TP_UYVY_640x480_Progressive_fps_60(self):
        pass
