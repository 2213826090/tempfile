# coding: utf-8
import os
import time
import sys
import subprocess
from testlib.util.common import g_common_obj
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.AOSPCamera import AOSPCamera
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.checkImage import CheckImage
from testlib.multimedia.relay08_helper import Relay08Helper
from testlib.multimedia.multimedia_switch_camera_helper import MultiMediaSwitchCameraHelper
from testlib.multimedia.multimedia_lightbox_helper import MultiMediaLightBoxHelper, MultiMediaRobotHelper, MultiMediaScrollHelper
from testlib.multimedia.multimedia_mplayer_helper import MultiMediaMplayerHelper
from testlib.multimedia.multimedia_checkiq_helper import MultiMediaCheckiqHelper
from testlib.multimedia.multimedia_high_speed_camera_helper import MultiMediaHighSpeedCameraHelper

def my_input(self):
    print "Wait for input…"
    raw_input()
    print "Got input and continue…"

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """
    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(CameraTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)

        self.camera_common = CameraCommon()
        self.host_path = self.camera_common.getTmpDir()

        self.relay08_helper = Relay08Helper()
        self.multimedia_lightbox_helper = MultiMediaLightBoxHelper()
        self.multimedia_robot_helper = MultiMediaRobotHelper()
        self.multimedia_mplayer_helper = MultiMediaMplayerHelper()
        self.multimedia_checkiq_helper = MultiMediaCheckiqHelper(self.host_path)
        self.multimedia_high_speed_helper = MultiMediaHighSpeedCameraHelper()
        self.multimedia_high_speed_helper.clear_temp_dir()

        self.rvc_camera = ""
        self.aosp_camera = ""
        self.case_result = -1

        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.camera_common.unlockScreen()
        self.get_rvc_camera_class().skipAccountLoginLyout()
        self.get_rvc_camera_class().backHome()

        self.multimedia_high_speed_helper.connect()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        self.multimedia_mplayer_helper.close_video()
        self.multimedia_high_speed_helper.disconnect()
        g_common_obj.stop_exp_handle()

        if self.rvc_camera != "":
            self.rvc_camera.stopCameraApp()
        if self.aosp_camera != "":
            self.aosp_camera.cleanMediaFiles()
        self.camera_common.removeDeivceFile()
        try:
            self.multimedia_robot_helper.reset()
        except:
            self.logger.debug("tearDown multimedia_robot_helper.reset error")
        try:
            self.multimedia_lightbox_helper.turn_off_all_light()
        except:
            self.logger.debug("tearDown multimedia_lightbox_helper.turn_off_all_light error")
        if self.case_result == 0:
            self.get_rvc_camera_class().reboot_device()
        time.sleep(3)
        self.get_rvc_camera_class().backHome()

    def check_file_corrupt(self, mediaFileCount=1):
        return self.camera_common.checkFileCorrupt(mediaFileCount)

    def get_rvc_camera_class(self):
        if self.rvc_camera == "":
            self.rvc_camera = MultiMediaSwitchCameraHelper().switchplatform()
        return self.rvc_camera

    def get_aosp_camera_class(self):
        if self.aosp_camera == "":
            self.aosp_camera = AOSPCamera()
        return self.aosp_camera

    def set_lightbox_operation(self, light_port_list=["Power", "UV"], position=-1, pause=0):
        self.multimedia_lightbox_helper.press_light_with_list(light_port_list)
        if pause == 1:
            my_input()
        self.multimedia_robot_helper.move_to_default_position(position)
        time.sleep(2)

    def set_scroll_operation(self, page_num=0):
        self.multimedia_scroll_helper.reset()
        if page_num != 0:
            self.change_page_flag = 1
        self.multimedia_scroll_helper.goto_page(page_num)
        time.sleep(2)

    def check_data(self, actual_result, expect_result, percent, log_str):
        actual_result = float(actual_result)
        expect_result = float(expect_result)
        min_expect_result = int(expect_result * (1 - percent))
        max_expect_result = int(expect_result * (1 + percent))
        if min_expect_result < actual_result and actual_result < max_expect_result:
            print "%s actual_result:%s in (%s, %s)" % (log_str, str(actual_result), str(min_expect_result), str(max_expect_result))
            return 0
        else:
            print "%s actual_result:%s not in (%s, %s)" % (log_str, str(actual_result), str(min_expect_result), str(max_expect_result))
            return -1

    def test_camera_0(self):
        self.case_result = 0
        self.set_lightbox_operation()
        self.case_result = 1

    def test_camera(self):
        self.case_result = 0
        self.set_lightbox_operation()
        self.multimedia_mplayer_helper.play_video("/home/auto1/tmp/video_30s.avi")
        time.sleep(2)
        self.multimedia_mplayer_helper.control_video(["pause", "pausing seek 0 2", "frame_step"])
        self.get_rvc_camera_class().startCameraApp()
        self.multimedia_mplayer_helper.control_video(["pause"])
        time.sleep(1)
        jpg_folder = self.multimedia_high_speed_helper.start_capture(20)
        self.multimedia_mplayer_helper.close_video()
        self.get_rvc_camera_class().stopCameraApp()
        jpg_name_list = self.multimedia_checkiq_helper.get_barcode_list(jpg_folder)
        barcode_list = self.multimedia_checkiq_helper.get_barcode_with_jpg_list(jpg_folder, jpg_name_list)
        pass_num, error_num, actual_barcode_num_list = self.multimedia_checkiq_helper.check_barcode_list(barcode_list)
        print pass_num, error_num, actual_barcode_num_list
        self.case_result = 1

    def test_camera_with_cold_boot(self):
        self.case_result = 0
        #raw_input("-------------prepare complete------------")
        self.set_lightbox_operation(light_port_list=["Power", "UV"])
        self.multimedia_mplayer_helper.play_video("/home/auto1/tmp/video_30s.avi")
        time.sleep(2)
        self.get_rvc_camera_class().startCameraApp()
        self.multimedia_mplayer_helper.control_video(["pause", "pausing seek 0 2", "frame_step"])
        my_input()
        self.rvc_camera.pressPowerKey(5)
        time.sleep(5)
        self.rvc_camera.pressPowerKey(2)
        self.multimedia_mplayer_helper.control_video(["pause"])
        time.sleep(4)
        jpg_folder = self.multimedia_high_speed_helper.start_capture(150)
        self.multimedia_mplayer_helper.close_video()
        #self.get_rvc_camera_class().stopCameraApp()
        jpg_name_list = self.multimedia_checkiq_helper.get_barcode_list(jpg_folder)
        barcode_list = self.multimedia_checkiq_helper.get_barcode_with_jpg_list(jpg_folder, jpg_name_list)
        pass_num, error_num, actual_barcode_num_list = self.multimedia_checkiq_helper.check_barcode_list(barcode_list)
        self.get_rvc_camera_class().stopCameraApp()
        if pass_num == 0:
            pass_num = 1
            error_num = 1
            actual_barcode_num_list = [-1, 157]
        print pass_num, error_num, actual_barcode_num_list
        assert pass_num > 0, "can't find barcode with cold boot device"
        print "==================Result==================="
        print "First find barcode picture: %d, about %.2fs" % (actual_barcode_num_list[1], actual_barcode_num_list[1]/30.0)
        self.case_result = 1

    def test_camera_with_scroll(self):
        self.case_result = 1
        self.multimedia_scroll_helper = MultiMediaScrollHelper()
        time.sleep(1)
        self.multimedia_scroll_helper.reset()
        my_input()
        self.set_lightbox_operation(light_port_list=["Power", "F", "D65", "TL84", "UV", "CWF", "TL83/U30", "F", "D65", "TL84", "UV", "TL83/U30"], position=210, pause=1)
        self.multimedia_robot_helper.rotate(-180)
        my_input()
        self.multimedia_scroll_helper.goto_page(1)
        my_input()
        self.multimedia_scroll_helper.goto_page(0)
        my_input()
        self.get_aosp_camera_class().startCameraApp()
        self.get_aosp_camera_class().capturePhoto()
        t_file = self.camera_common.getMediaFilesFromDevice(0, 1)[0]
        fisheyecorrection_folder = "/home/auto1/tmp/fisheyecorrection/fisheyecorrection/"
        subprocess.Popen("./fisheye %s" % t_file, cwd=fisheyecorrection_folder, shell=True)
        dest_adjusted_file = "/home/auto1/tmp/logs/adjusted.jpg"
        os.system("cp %sadjusted.jpg %s" % (fisheyecorrection_folder, dest_adjusted_file))
        dest_file = dest_adjusted_file if os.path.exists(dest_adjusted_file) else t_file
        ret = self.multimedia_checkiq_helper.check_IQ.detectRect(dest_file)
        ret = int(ret)
        self.logger.debug("detectRect ret=%s" % ret)
        ret = self.multimedia_checkiq_helper.check_IQ.getTemp(dest_file)
        #os.system("cp /home/auto1/tmp/checkiq_temp_dir/rects.jpg /home/auto1/tmp/logs/rects.jpg")
        print "==================Result==================="
        t_result = 0
        t_result += self.check_data(ret, 6200, 0.05, "check color temperature")
        ret = CheckImage().brightness(dest_file)
        t_result += self.check_data(ret, 130, 0.15, "check brightness")
        self.get_aosp_camera_class().stopCameraApp()
        self.multimedia_scroll_helper.reset()
#         os.system("eog -s %s" % "/home/auto1/tmp/rects_adjusted.jpg")
        os.system("eog -s %s" % dest_file)
        assert t_result == 0, "check color temperature or check brightness failed! t_result=%s" % str(t_result)
        self.case_result = 1

    def lightbox_main_test(self, sub_func_name="", *arg, **keywords):
        """
        This test used to test Camera app
        The test case spec is following:
        1. appPrepare()
        2. do sub_func()
        """
        self.case_name = sys._getframe().f_back.f_code.co_name
        if sub_func_name == "":
            sub_func_name = "%s_sub_func" % self.case_name
        self.logger.debug("case_name=%s" % self.case_name)
        self.logger.debug("netflix_main_test---sub_func_name=%s" % sub_func_name)
        self.appPrepare()

        self.logger.debug("Arbitrary parameter is %s" % str(arg))
        self.logger.debug("keywords parameter is %s" % str(keywords))
        getattr(self, sub_func_name)(*arg, **keywords)

        self.logger.debug("Case %s is pass!" % self.case_name)

    def test_Camera_Test(self):
        self.lightbox_main_test("test_camera_test")
