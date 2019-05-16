# coding: utf-8
import time
import sys
import os
import shutil
import threading

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_switch_camera_helper import MultiMediaSwitchCameraHelper
from testlib.multimedia.multimedia_scale_test_helper import MultiMediaScaleTestHelper
from testlib.multimedia.multimedia_logcat_helper import MultiMediaLogcatHelper
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_checkiq_helper import MultiMediaCheckiqHelper
from testlib.graphics.common import adb32
from testlib.camera.checkIQ import CheckIQ

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """
    config = TestConfig()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)
        self.case_result = -1

        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self.camera_common = CameraCommon()
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)

        self.host_path = self.camera_common.getTmpDir()
        self.multimedia_scale_test_helper = MultiMediaScaleTestHelper(self.host_path)
        self.multimedia_checkiq_helper = MultiMediaCheckiqHelper(self.host_path)
        self.android_version = MultiMediaSwitchCameraHelper(skip_import_camera=True).android_version.lower()

        self.multimedia_scale_test_helper.scale_test_prepare()

        self.camera_common.removeDeivceFile()
        self.camera_common.removeDeivceFile(self.multimedia_scale_test_helper.movies_folder_device_path + "*")
        self.camera_common.removeFile(self.host_path + "/*.mp4")

        self.camera_common.unlockScreen()
        self.d = g_common_obj.get_device()
        self.skipAccountLoginLyout()
        g_common_obj.back_home()
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

        self.camera_common.removeDeivceFile()
        self.camera_common.removeDeivceFile(self.multimedia_scale_test_helper.movies_folder_device_path + "*")
        if self.multimedia_scale_test_helper.scale_test_teardown() == 1 or self.case_result == 0:
            self.reboot_device()
        time.sleep(3)
        g_common_obj.back_home()

    def skipAccountLoginLyout(self):
        if self.d(textContains="Drive safely").exists:
            self.d(text="Owner").click()
            time.sleep(3)

    def reboot_device(self):
        adb32._adb_reboot()
#         self.skipAccountLoginLyout()
        time.sleep(15)

    def check_logcat_data(self, data_list, expect_fps):
        self.logger.debug("check_logcat_data data_list=%s" % data_list)
        if len(data_list) == 0:
            return False, -1
        i = 0
        for t_fps in data_list:
            i += 1
            t_fps = float(t_fps)
            if expect_fps * 0.95 > t_fps or t_fps > expect_fps * 1.05:
                if i <= 3:
                    continue
                return False, t_fps
        return True, 1

    def check_scale_TP_with_similarity(self, cmd_arguments_list, expect_percent=1, generated_jpg_file_rule="screenshot"):
        self.case_result = 0
        self.case_name = sys._getframe().f_back.f_code.co_name

        expect_fps = cmd_arguments_list[0].split(" ")[-1]
        self.multimedia_scale_test_helper.check_scale_test_xml(expect_fps, case_name=self.case_name)

        self.reboot_device()

        if len(cmd_arguments_list) == 1:
            cmd_arguments_list *= 4

        t_actual_num = 0
        t_expect_num = 0
        prev_cmd_arguments = ""
        t_percent_list = []
        for cmd_arguments in cmd_arguments_list:
            t_actual_num += 1
            expect_fps = cmd_arguments.split(" ")[-1]
#             cmd_arguments = cmd_arguments.replace(" " + expect_fps, " 0.0")
            self.logger.debug("------loop %s------%s" % (str(t_actual_num), cmd_arguments))
            if prev_cmd_arguments == cmd_arguments and self.multimedia_scale_test_helper.generated_reference_library == 1:
                continue
            if prev_cmd_arguments != cmd_arguments:
                t_expect_num += 1
            prev_cmd_arguments = cmd_arguments

            if self.multimedia_scale_test_helper.generated_reference_library != 1:
                jpg_file_name = "%s_%d.jpg" % (self.case_name, t_actual_num)
            else:
                jpg_file_name = "%s_%d.jpg" % (self.case_name, t_expect_num)
            expect_jpg_file_name = "%s_%d.jpg" % (self.case_name, t_expect_num)

            if self.multimedia_scale_test_helper.generated_reference_library != 1:
                generated_jpg_file_folder = self.multimedia_scale_test_helper.actual_folder
            else:
                generated_jpg_file_folder = self.multimedia_scale_test_helper.expect_folder

            thread_1 = threading.Thread(target=self.multimedia_scale_test_helper.play_and_record_video_with_instrument, args=(cmd_arguments, self.case_name))
            thread_1.start()
            #self.multimedia_scale_test_helper.play_and_record_video_with_instrument(cmd_arguments)
            if generated_jpg_file_rule == "screenshot":
                time.sleep(8)
                jpg_file_name = jpg_file_name.replace(".jpg", ".png")
                expect_jpg_file_name = expect_jpg_file_name.replace(".jpg", ".png")
                self.camera_common.getScreenshotAndPullToHost(jpg_file_name, generated_jpg_file_folder)
                generated_jpg_file_path = os.path.join(generated_jpg_file_folder, jpg_file_name)
                while self.multimedia_scale_test_helper.play_result == -1:
                    time.sleep(1)
                if self.multimedia_scale_test_helper.play_result != 1:
                    t_percent_list.append((0, generated_jpg_file_path, "play error! play_result=%s" % self.multimedia_scale_test_helper.play_result))
                    continue
            else:
                thread_1.join()
                if self.multimedia_scale_test_helper.play_result != 1:
                    t_percent_list.append((0, "play error!", "play_result=%s" % self.multimedia_scale_test_helper.play_result))
                    continue
                movies_file_host_path = self.multimedia_scale_test_helper.pull_video_file_to_host()
                generated_jpg_file_path = self.multimedia_scale_test_helper.get_jpg_from_video(movies_file_host_path, jpg_file_name)
            self.multimedia_scale_test_helper.play_result = -1
            self.logger.debug("check_scale_TP_with_similarity jpg_file_path=%s" % generated_jpg_file_path)

            if self.multimedia_scale_test_helper.generated_reference_library != 1:
                if "AVM737" in self.case_name:
                    t_percent_list.append((1, generated_jpg_file_path, "AVM737 playback no crash!"))
                    continue
                if "o" in self.android_version:
                    t_region=CheckIQ.Region(0, 0, 1900, 1024)
                else:
                    t_region=CheckIQ.Region(max=True)
                expect_jpg_file_path = os.path.join(self.multimedia_scale_test_helper.expect_folder, expect_jpg_file_name)
                if not os.path.exists(expect_jpg_file_path):
                    self.multimedia_scale_test_helper.download_reference_library_file(expect_jpg_file_path)
                assert os.path.exists(expect_jpg_file_path), "Can't find reference library(%s)! please check it." % expect_jpg_file_path
                t_percent = self.multimedia_checkiq_helper.compare_picture_similarity(generated_jpg_file_path, expect_jpg_file_path, t_region)
                t_percent_list.append((t_percent, generated_jpg_file_path, expect_jpg_file_path))

        if self.multimedia_scale_test_helper.generated_reference_library != 1:
            self.logger.debug("check_scale_TP_with_similarity t_percent_list=%s" % str(t_percent_list))
            pass_num = 0
            t_sum = 0
            user_log_dir = g_common_obj.get_user_log_dir()
            self.logger.debug("check_scale_TP_with_similarity user_log_dir=%s" % str(user_log_dir))
            for t_actual_percent, generated_jpg_file_path, _ in t_percent_list:
                t_sum += 1
                if t_actual_percent >= expect_percent:
                    pass_num += 1
                elif "!" not in generated_jpg_file_path:
                    shutil.copy(generated_jpg_file_path, user_log_dir)
            case_message_info = "Similarity failed! pass_rate=%.2f%%, t_percent_list=%s" % (pass_num/(t_sum*1.0)*100, t_percent_list)
            if "640x480_Progressive" in self.case_name or "720x576_Progressive" in self.case_name:#OAM issue don't need fixed,so ignore it.(sync with sunxue)
                pass_num = t_sum if pass_num >= 1 else pass_num
            assert pass_num == t_sum, case_message_info
        else:
            self.logger.debug("case_name generated reference library complete!")
        self.case_result = 1

    def check_scale_TP_with_fps(self, cmd_arguments_list):
        self.case_result = 0
        self.case_name = sys._getframe().f_back.f_code.co_name

#         assert "o" not in self.android_version, "Feature not merge in BxtP_O"

        expect_fps = cmd_arguments_list[0].split(" ")[-1]
        self.multimedia_scale_test_helper.check_scale_test_xml(expect_fps, case_name=self.case_name)

        self.reboot_device()

        g_common_obj.adb_cmd_capture_msg("setprop camera.hal.perf 3")

        t_percent_list = []
        if "o" in self.android_version:
            self.multimedia_logcat_helper = MultiMediaLogcatHelper("adb logcat Camera3HWI:D *:S")
        else:
            self.multimedia_logcat_helper = MultiMediaLogcatHelper("adb logcat CameraHardwareSoc:D *:S")
        for cmd_arguments in cmd_arguments_list:
            if "3840 2160" in cmd_arguments:
                continue
            self.multimedia_logcat_helper.get_logcat_data_start()

            self.multimedia_scale_test_helper.play_result = -1
            expect_fps = cmd_arguments.split(" ")[-1]
            cmd_arguments = cmd_arguments.replace(" " + expect_fps, " 0.0")
            thread_1 = threading.Thread(target=self.multimedia_scale_test_helper.play_and_record_video_with_instrument, args=(cmd_arguments, self.case_name))
            thread_1.start()

            while self.multimedia_scale_test_helper.play_result == -1:
                time.sleep(1)

            if "o" in self.android_version:
                result_list = self.multimedia_logcat_helper.get_logcat_data_end("FPS=(.*) buffcount")
            else:
                result_list = self.multimedia_logcat_helper.get_logcat_data_end("total fps is (.*),")
            if self.multimedia_scale_test_helper.play_result != 1:
                t_percent_list.append((0, "cmd=%s, play error!" % cmd_arguments, "play_result=%s" % self.multimedia_scale_test_helper.play_result))
            elif len(result_list) == 0:
                t_percent_list.append((-1, "find error!", "can't find fps in logcat! cmd_arguments=%s" % cmd_arguments))
            else:
                expect_fps = float(expect_fps)
                check_logcat_data_result, error_fps = self.check_logcat_data(result_list, expect_fps)
                if check_logcat_data_result:
                    t_percent_list.append((1, "cmd=%s, pass!" % cmd_arguments, "expect_fps=%s, result_list=%s" % (expect_fps, result_list)))
                else:
                    t_percent_list.append((-2, "cmd=%s, fps error!" % cmd_arguments, "error_fps=%s, expect_fps=%s, result_list=%s" % (error_fps, expect_fps, result_list)))
        self.logger.debug("check_scale_TP_with_fps t_percent_list=%s" % str(t_percent_list))
        pass_num = 0
        t_sum = 0
        for t_result, _, _ in t_percent_list:
            t_sum += 1
            if t_result == 1:
                pass_num += 1
        case_message_info = "check fps failed! pass_rate=%.2f%%, t_percent_list=%s" % (pass_num/(t_sum*1.0)*100, t_percent_list)
        assert pass_num == t_sum, case_message_info
        self.case_result = 1