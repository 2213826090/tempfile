# coding: UTF-8
'''
Created on May 9, 2017

@author: Li Zixi
'''
import os
import re
import time
import subprocess
import shutil
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, verify_apps
from testlib.multimedia.get_device_port_helper import GetConfigFileHelper
from testlib.multimedia.multimedia_switch_camera_helper import MultiMediaSwitchCameraHelper
from testlib.util.log import Logger

class MultiMediaScaleTestHelper:
    def __init__(self, host_path):
        self.logger = Logger.getlogger()
        self.host_path = host_path
        self.camera_helper = MultiMediaSwitchCameraHelper(skip_import_camera=True)
        self.target_folder = os.path.join(self.host_path, self.camera_helper.device, self.camera_helper.android_version)
        self.expect_folder = os.path.join(self.target_folder, "expect")
        self.actual_folder = os.path.join(self.target_folder, "actual")
        if not os.path.exists(self.expect_folder):
            os.makedirs(self.expect_folder)
        if not os.path.exists(self.actual_folder):
            os.makedirs(self.actual_folder)

        self.ffmpeg_cmd = "ffmpeg -i %s -ss 00:00.02 -y -q:v 2 -vframes 1 %s"
        self.movies_folder_device_path = "/sdcard/Movies/"
        self.scale_test_xml_value_dict = {1:30, 2:50, 3:60}
        self.play_result = -1

        self.get_cfg_file_helper = GetConfigFileHelper("", "multimedia_scale_test_helper.conf")
        self.scale_test_cfg = self.get_cfg_file_helper.get_section("config")
        self.multimedia_setting = MultiMediaSetting(self.get_cfg_file_helper.cfg_file)
        self.generated_reference_library = int(self.scale_test_cfg.get("generated_reference_library"))
        self.reference_library_folder = self.scale_test_cfg.get("reference_library_folder")
        self.reference_library_folder = os.path.join(self.reference_library_folder, self.camera_helper.device, self.camera_helper.android_version)
        self.xml_file_dst_path = self.scale_test_cfg.get("xml_file_dst_path")
        self.o_image_camera3_xml_file_dst_path = self.scale_test_cfg.get("o_image_camera3_xml_file_dst_path")

    def __execute_command_with_popen(self, cmd, t_shell=False):
        self.logger.debug("__execute_command_with_popen cmd=%s" % cmd)
        if not t_shell:
            cmd = cmd.split()
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=t_shell)

    def remount_device(self):
        if self.camera_helper.android_version == "o":
            from testlib.graphics.common import adb32
            adb32.adb_disable_verity()
        else:
            from testlib.common.common import g_common_obj2
            self.serial = g_common_obj2.getSerialNumber()
            os.system("adb -s %s root" %self.serial)
            os.system("adb -s %s root" %self.serial)
            os.system("adb -s %s disable-verity" %self.serial)
            os.system("adb -s %s reboot" %self.serial)
            time.sleep(60)
            os.system("adb -s %s root" %self.serial)
            time.sleep(30)
            os.system("adb -s %s root" %self.serial)
            result = 'remount succeeded'
            remount_result = os.popen("adb -s %s remount" %self.serial).read().strip()
            print "[Info] --- remount result is %s" %remount_result
            if result == remount_result:
                print "[Info] --- remount successfull"
            else:
                assert False, "[Info] --- remount fail"

    def backup_file(self, dst_path_string, backup_name_postfix="_backup"):
        dst_path = self.scale_test_cfg.get(dst_path_string)
        dst_path_root, dst_path_ext = os.path.splitext(dst_path)
        backup_dst_path = "%s%s%s" % (dst_path_root, backup_name_postfix, dst_path_ext)
        self.logger.debug("backup_file backup_dst_path=%s" % backup_dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % backup_dst_path)
        if backup_dst_path not in t_result:
            g_common_obj.adb_cmd_capture_msg("cp %s %s" % (dst_path, backup_dst_path))
            t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % backup_dst_path)
            assert backup_dst_path in t_result, "cp failed! t_result=%s" % t_result
        return (dst_path, backup_dst_path)

    def restore_file(self, dst_path_string, backup_name_postfix="_backup"):
        dst_path = self.scale_test_cfg.get(dst_path_string)
        dst_path_root, dst_path_ext = os.path.splitext(dst_path)
        backup_dst_path = "%s%s%s" % (dst_path_root, backup_name_postfix, dst_path_ext)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % backup_dst_path)
        if backup_dst_path in t_result:
            g_common_obj.adb_cmd_capture_msg("cp %s %s" % (backup_dst_path, dst_path))
            t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % dst_path)
            self.logger.debug("backup_file t_result=%s" % t_result)
            return True
        return False

    def replace_file(self, src_path_string, dst_path_string):
        src_path = self.scale_test_cfg.get(src_path_string)
        dst_path = self.scale_test_cfg.get(dst_path_string)
        g_common_obj.adb_cmd_capture_msg("rm -rf %s" % dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % dst_path)
        assert dst_path not in t_result, "rm failed! t_result=%s" % t_result
        dst_path = self.multimedia_setting.push_file_new(src_path, dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % dst_path)
        assert dst_path in t_result, "push failed! t_result=%s" % t_result
        return dst_path

    def scale_test_o_image_prepare(self):
        package_name, _ = self.multimedia_setting.get_package_and_activity_name("for_o_image_camera_preview_test_apk")
        self.o_image_play_cmd = self.scale_test_cfg.get("o_image_play_cmd")
        result = verify_apps(package_name)

        self.remount_device()
        self.backup_file("o_image_media_xml_file_dst_path")
        self.o_image_media_xml_file_dst_path = self.replace_file("o_image_media_xml_file_src_path", "o_image_media_xml_file_dst_path")
        self.backup_file("o_image_libcamhal_xml_file_dst_path")
        self.o_image_libcamhal_xml_file_dst_path = self.replace_file("o_image_libcamhal_xml_file_src_path", "o_image_libcamhal_xml_file_dst_path")
        self.o_image_camera3_xml_file_dst_path = self.backup_file("o_image_camera3_xml_file_dst_path")[0]
        self.restore_file("o_image_camera3_xml_file_dst_path", "_hdmi")

        if not result:
            self.multimedia_setting.install_apk("for_o_image_camera_preview_test_apk")
        return True

    def scale_test_default_image_prepare(self, fps="60fps"):
        self.sh_file_src_path = self.scale_test_cfg.get("sh_file_src_path")
        self.sh_file_dst_path = self.scale_test_cfg.get("sh_file_dst_path")
        self.backup_file("xml_file_dst_path")
        package_name, _ = self.multimedia_setting.get_package_and_activity_name("camera_preview_test_apk")
        result = verify_apps(package_name)
        if not result:
            self.remount_device()

            g_common_obj.adb_cmd_capture_msg("rm -rf %s" % self.sh_file_dst_path)
            t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % self.sh_file_dst_path)
            assert "No such file or directory" in t_result, "rm failed! t_result=%s" % t_result
            self.sh_file_dst_path = self.multimedia_setting.push_file_new(self.sh_file_src_path, self.sh_file_dst_path)
            t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % self.sh_file_dst_path)
            assert "No such file or directory" not in t_result, "push failed! t_result=%s" % t_result
            g_common_obj.adb_cmd_capture_msg("chmod 777 %s" % self.sh_file_dst_path)

            self.multimedia_setting.install_apk("camera_preview_test_apk")
        return not result

    def scale_test_prepare(self, fps="60fps"):
        self.logger.debug("scale_test_prepare")
        if self.camera_helper.android_version == "o":
            result = self.scale_test_o_image_prepare()
        else:
            result = self.scale_test_default_image_prepare(fps)
        return result

    def scale_test_o_image_teardown(self):
        self.remount_device()
        self.restore_file("o_image_media_xml_file_dst_path")
        self.restore_file("o_image_libcamhal_xml_file_dst_path")
        self.restore_file("o_image_camera3_xml_file_dst_path")
        return 1

    def scale_test_teardown(self):
        self.logger.debug("scale_test_teardown")
        self.remount_device()
        if self.camera_helper.android_version == "o":
            self.restore_file("o_image_media_xml_file_dst_path")
            self.restore_file("o_image_libcamhal_xml_file_dst_path")
            self.restore_file("o_image_camera3_xml_file_dst_path")
        else:
            self.restore_file("xml_file_dst_path")
        return 1

    def change_scale_test_xml(self, fps):
        fps = str(fps)
        self.logger.debug("change_scale_test_xml fps=%s" % fps)
        if "fps" not in fps and "default" not in fps:
            fps = fps+"fps"
        self.remount_device()
        xml_file_src_path = self.scale_test_cfg.get("%s_xml_file_src_path" % fps)
        xml_file_dst_path = self.scale_test_cfg.get("%s_xml_file_dst_path" % fps)
        g_common_obj.adb_cmd_capture_msg("rm -rf %s" % xml_file_dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % xml_file_dst_path)
        assert "No such file or directory" in t_result, "rm failed! t_result=%s" % t_result
        self.xml_file_dst_path = self.multimedia_setting.push_file_new(xml_file_src_path, xml_file_dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % self.xml_file_dst_path)
        assert self.xml_file_dst_path in t_result, "push failed! t_result=%s" % t_result

    def change_scale_test_xml_param_handle(self, param, str1, str2):
        return " -e 's#%s#%s#g' " % (param % str1, param % str2)

    def change_scale_test_xml_for_o_image(self, dst_path, case_name):
        self.logger.debug("change_scale_test_xml_for_o_image")
        field, input_size, input_format, deinterlace_mode = self.analyze_case_name(case_name)
        main_param = ""
        main_param += self.change_scale_test_xml_param_handle("intel.sensorprop.field\ value=\\\"%s\\\"", ".*", field)
        main_param += self.change_scale_test_xml_param_handle("intel.sensorprop.inputSize\ value=\\\"%s\\\"", ".*", input_size)
        main_param += self.change_scale_test_xml_param_handle("intel.sensorprop.inputFormat\ value=\\\"%s\\\"", ".*", input_format)
        main_param += self.change_scale_test_xml_param_handle("intel.sensorprop.deinterlaceMode\ value=\\\"%s\\\"", ".*", deinterlace_mode)
        cmd = "sed -i %s %s" % (main_param, dst_path)
        self.logger.debug("change_scale_test_xml_for_o_image cmd=%s" % cmd)
        g_common_obj.adb_cmd_capture_msg(cmd)

    def change_libcamhal_xml_for_o_image(self, dst_path, case_name):
        self.logger.debug("change_libcamhal_xml_for_o_image")
        actual_fps = self.check_string_with_regular_expression(case_name, r"_(\d*)[fps]{0,3}$", "")[1][0]
        for t_key in self.scale_test_xml_value_dict.keys():
            if actual_fps == str(self.scale_test_xml_value_dict[t_key]):
                break
        main_param = ""
        main_param += self.change_scale_test_xml_param_handle("ctrlId=\\\"V4L2_CID_TEST_PATTERN\\\"\ value=\\\"%s\\\"\ ctrlName=", ".*", str(t_key))
        cmd = "sed -i %s %s" % (main_param, dst_path)
        self.logger.debug("change_libcamhal_xml_for_o_image cmd=%s" % cmd)
        g_common_obj.adb_cmd_capture_msg(cmd)

    def check_string_with_regular_expression(self, string, parttern, expect_value):
        get_value_parttern = re.compile(parttern)
        value_list = get_value_parttern.findall(string)

        assert len(value_list) != 0, "Can't find parttern(%s) in string!" % parttern

        actual_value = value_list[0]
        self.logger.debug("check_string_with_regular_expression actual_value=%s, expect_value=%s" % (actual_value, expect_value))
        result = 0 if actual_value == expect_value else -1
        return (result, value_list)

    def analyze_case_name(self, case_name):
        field = "ANY" if "progressive" in case_name.lower() else "ALTERNATE"
        deinterlace_mode = "OFF" if "progressive" in case_name.lower() else "ON"
        input_size = self.check_string_with_regular_expression(case_name, r"_(\d*x\d*)", "")[1][0]
        input_size = input_size.replace("x", ",")
        input_format = "YUY2" if "YUYV" in case_name else "UYVY"

        self.logger.debug("analyze_case_name field=%s, input_size=%s, input_format=%s, deinterlace_mode=%s" % (field, input_size, input_format, deinterlace_mode))
        return field, input_size, input_format, deinterlace_mode

    def check_scale_test_xml_for_o_image(self, case_name):
        self.logger.debug("check_scale_test_xml_for_o_image case_name=%s" % case_name)
        field, input_size, input_format, deinterlace_mode = self.analyze_case_name(case_name)
        check_scale_test_xml_cmd = "cat %s" % self.o_image_camera3_xml_file_dst_path
        t_result = g_common_obj.adb_cmd_capture_msg(check_scale_test_xml_cmd)

        result = 0
        result += self.check_string_with_regular_expression(t_result, r"<intel.sensorprop.field value=\"(.*)\" />", field)[0]
        result += self.check_string_with_regular_expression(t_result, r"<intel.sensorprop.inputSize value=\"(.*)\" />", input_size)[0]
        result += self.check_string_with_regular_expression(t_result, r"<intel.sensorprop.inputFormat value=\"(.*)\" />", input_format)[0]
        result += self.check_string_with_regular_expression(t_result, r"<intel.sensorprop.deinterlaceMode value=\"(.*)\" />", deinterlace_mode)[0]
        return True if result == 0 else False

    def check_libcamhal_xml_for_o_image(self, case_name):
        self.logger.debug("check_libcamhal_xml_for_o_image case_name=%s" % case_name)
        check_libcamhal_xml_cmd = "cat %s" % self.o_image_libcamhal_xml_file_dst_path
        t_result = g_common_obj.adb_cmd_capture_msg(check_libcamhal_xml_cmd)
        actual_fps = self.check_string_with_regular_expression(case_name, r"_(\d*)[fps]{0,3}$", "")[1][0]
        for t_key in self.scale_test_xml_value_dict.keys():
            if actual_fps == str(self.scale_test_xml_value_dict[t_key]):
                break
        result = 0
        result += self.check_string_with_regular_expression(t_result, r"ctrlId=\"V4L2_CID_TEST_PATTERN\" value=\"(.*)\" ctrlName=", str(t_key))[0]
        return True if result == 0 else False

    def check_scale_test_xml_for_default_image(self, expect_fps, special_str=""):
        expect_fps = str(expect_fps)
        if special_str != "":
            expect_fps = "default"
        check_scale_test_xml_cmd = "cat %s" % self.xml_file_dst_path
        t_result = g_common_obj.adb_cmd_capture_msg(check_scale_test_xml_cmd)
        get_scale_xml_fps_value_parttern = re.compile(r"<control name=\"adv7481-hdmi pixel array 0-00e0\".*value=\"(.)\".*/>")
        fps_value_result_list = get_scale_xml_fps_value_parttern.findall(t_result)
        self.logger.debug("check_scale_test_xml fps_value_result_list=%s" % fps_value_result_list)
        if len(fps_value_result_list) == 0:
            actual_fps = "default"
        else:
            fps_value_result = fps_value_result_list[0]
            self.logger.debug("check_scale_test_xml fps_value_result=%s" % fps_value_result)
            actual_fps = str(self.scale_test_xml_value_dict[int(fps_value_result[0])])
        self.logger.debug("check_scale_test_xml actual_fps=%s, expect_fps=%s" % (actual_fps, expect_fps))
        if actual_fps != expect_fps:
            self.change_scale_test_xml(expect_fps)
            return True
        else:
            return False

    def check_scale_test_xml(self, expect_fps, case_name=""):
        if self.camera_helper.android_version == "o":
            need_reboot = False
            if "AVM737" in case_name:
                need_reboot = True
                self.o_image_libcamhal_xml_file_dst_path = self.replace_file("o_image_avm737_libcamhal_xml_file_src_path", "o_image_avm737_libcamhal_xml_file_dst_path")
            elif "fps" in case_name.lower():
                result = self.check_libcamhal_xml_for_o_image(case_name)
                if not result:
                    need_reboot = True
                    self.change_libcamhal_xml_for_o_image(self.o_image_libcamhal_xml_file_dst_path, case_name)
                    assert self.check_libcamhal_xml_for_o_image(case_name), "Fail change libcamhal xml!"

            result = self.check_scale_test_xml_for_o_image(case_name)
            if not result:
                need_reboot = True
                self.change_scale_test_xml_for_o_image(self.o_image_camera3_xml_file_dst_path, case_name)
                assert self.check_scale_test_xml_for_o_image(case_name), "Fail change scale test xml!"

            return need_reboot
        else:
            special_str = "AVM737" if "AVM737" in case_name else ""
            result = self.check_scale_test_xml_for_default_image(expect_fps, special_str)
            return result

    def download_reference_library_file(self, file_path):
        file_name = os.path.split(file_path)[1]
        t_file_path = self.multimedia_setting.download_file_to_host(os.path.join(self.reference_library_folder, file_name))
        shutil.copyfile(t_file_path, file_path)
        return file_path

    def pull_video_file_to_host(self):
        movies_file_name = g_common_obj.adb_cmd_capture_msg("ls %s" % self.movies_folder_device_path)
        self.logger.debug("play_and_record_video_with_instrument movies_file_device_path=%s" % movies_file_name)
        g_common_obj.adb_cmd_common("pull %s/%s %s/%s" % (self.movies_folder_device_path, movies_file_name, self.host_path, movies_file_name))
        movies_file_host_path = os.path.join(self.host_path, movies_file_name)
        return movies_file_host_path

    def play_and_record_video_with_instrument(self, cmd_arguments, case_name):
        duration = 12
        self.play_result = -1
        if self.camera_helper.android_version == "o":
            self.logger.debug("play_and_record_video_with_instrument case_name=%s" % case_name)
            if "fps" in case_name.lower():
                cmd_arguments_list = cmd_arguments.split(" ")
                width, height = cmd_arguments_list[4], cmd_arguments_list[5]
            else:
                output_size = self.check_string_with_regular_expression(case_name, r"_(\d*x\d*)", "")[1][1]
                width, height = output_size.split("x")
            t_format = "NV21" if "NV21" in case_name else "NV21" #need extend
            cmd_result = g_common_obj.adb_cmd_capture_msg(self.o_image_play_cmd % (width, height, t_format, duration))
        else:
            self.logger.debug("play_and_record_video_with_instrument cmd=%s %s" % (self.sh_file_dst_path, cmd_arguments))
            cmd_result = g_common_obj.adb_cmd_capture_msg("%s %s" % (self.sh_file_dst_path, cmd_arguments))
        self.logger.debug("play_and_record_video_with_instrument cmd_result = %s" % cmd_result)
        if "OK" not in cmd_result:
            self.play_result = 0
        else:
            self.play_result = 1

    def get_jpg_from_video(self, video_file_path, output_file_name="output.jpg", output_folder=""):
        if output_folder == "":
            if self.generated_reference_library == 1:
                output_folder = self.expect_folder
            else:
                output_folder = self.actual_folder
        output_file_path = os.path.join(output_folder, output_file_name)
        t_ffmpeg_cmd = self.ffmpeg_cmd % (video_file_path, output_file_path)
        self.logger.debug("t_ffmpeg_cmd = %s" % t_ffmpeg_cmd)
        result = self.__execute_command_with_popen(t_ffmpeg_cmd).stdout.read()
        assert "Invalid" not in result and "error" not in result, "Generate jpg failed! result=%s" % result
        return output_file_path
