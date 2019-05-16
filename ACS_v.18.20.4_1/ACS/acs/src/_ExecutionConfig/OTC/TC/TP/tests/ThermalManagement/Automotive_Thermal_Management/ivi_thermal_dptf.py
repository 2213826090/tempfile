#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.thermal import Thermal
from testlib.em.constants_def import BXT_O
from testlib.em.tools import *

class IVI_DPTF(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal_file_remote = "/mnt/sdcard/thermal.txt"
        self.thermal_file_host = ""
        self.wait_time = 600
        self.score_list = []
        self.thermal = Thermal()
        self.thermal.adb_root()
        self.thermal.unlock_screen()
        g_common_obj.close_background_apps()
        super(IVI_DPTF, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.thermal.ivi_end_capture_thermal_data()
        g_common_obj.close_background_apps()
        super(IVI_DPTF, self).tearDown()

    def check_glbenchmark_result(self):
        score = self.glb.get_score()
        if score or not self.glb.check_running():
            if score:
                self.score_list.append(score)
            self.glb.clear_data()
            self.glb.launch()
            self.glb.select_offscreen_etc1_test()
            self.glb.run_tests()

    def keep_running_glbenchmark(self, timeout):
        stop_time = time.time() + timeout
        cycle = 1
        while time.time() < stop_time:
            print "Cycle:", cycle
            self.check_glbenchmark_result()
            time.sleep(20)
            cycle += 1

    def check_antutu_result(self):
        score = self.antutu.get_score()
        if score or not self.antutu.check_running():
            if score:
                self.score_list.append(score)
            self.antutu.run_test()

    def keep_running_antutu(self, timeout):
        stop_time = time.time() + timeout
        cycle = 1
        while time.time() < stop_time:
            print "Cycle:", cycle
            self.check_antutu_result()
            time.sleep(20)
            cycle += 1

    def verify_thermal_in_scenarial(self, scenarial):
        self.thermal_file_host = gen_tmp_file_path_by_time(scenarial + ".txt")
        self.thermal_data_result = gen_tmp_file_path_by_time(scenarial + ".xls")
        self.thermal.ivi_start_capture_thermal_data(self.thermal_file_remote)
        if scenarial == "Antutu":
            self.keep_running_antutu(self.wait_time)
        elif scenarial == "GLBenchmark":
            self.keep_running_glbenchmark(self.wait_time)
        else:
            time.sleep(self.wait_time)
        self.thermal.ivi_end_capture_thermal_data()
        g_common_obj.pull_file(self.thermal_file_host, self.thermal_file_remote)
        self.thermal.ivi_gen_temp_result(self.thermal_data_result, self.thermal_file_host, self.score_list)

    def ivi_capture_thermal_data(self):
        cpu_temp = self.thermal.ivi_get_cpu_temp()
        cpu_freq = self.thermal.ivi_get_cpu_freq()
        power = self.thermal.ivi_get_power()
        gpu_freq = self.thermal.get_gpu_freq()
        if self.thermal.get_product() == BXT_O:
            return "%s %s %s %s\n" % (cpu_temp, cpu_freq, power, gpu_freq)
        board_temp = self.thermal.ivi_get_board_temp()
        return "%s %s %s %s %s\n" % (cpu_temp, board_temp, cpu_freq, power, gpu_freq)

    def test_ivi_thermal_glbenchmark(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import GLBenchmark
        self.glb = GLBenchmark()
        self.glb.install()
        self.ivi_capture_thermal_data()
        self.verify_thermal_in_scenarial("GLBenchmark")

    def test_ivi_thermal_antutu(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Antutu4
        self.antutu = Antutu4()
        self.antutu.install()
        self.antutu.reject_network_access()
        self.ivi_capture_thermal_data()
        self.verify_thermal_in_scenarial("Antutu")

    def test_ivi_thermal_play_video(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import VideoPlayer
        self.player = VideoPlayer()
        self.player.install()
        video_file = self.player.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.ivi_capture_thermal_data()
        self.player.play_local_video(video_file)
        self.verify_thermal_in_scenarial("Video_playback")

    def test_ivi_thermal_video_recording(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Camera
        self.camera = Camera()
        self.camera.grant_permissions()
        self.ivi_capture_thermal_data()
        self.camera.launch("Video")
        self.camera.click_shutter_button("Video")
        self.verify_thermal_in_scenarial("Video_recording")

    def test_ivi_thermal_video_streaming(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import VideoPlayer
        from testlib.em.settings import WifiSetting
        WifiSetting().connect_wifi_by_conf("wifi_adb")
        self.player = VideoPlayer()
        self.ivi_capture_thermal_data()
        video_url = "http://" + get_server_ip() + get_config_value("webpage", "video_streaming")

        self.player.play_http_video(video_url)
        self.verify_thermal_in_scenarial("Video_streaming")

    def test_ivi_thermal_gps_audio_playback(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Maps, CleanMusic
        audio_file = self.thermal.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        Maps().launch()
        music = CleanMusic()
        music.install()
        self.ivi_capture_thermal_data()
        music.play_audio("file://" + audio_file)
        self.verify_thermal_in_scenarial("Maps")

    def test_ivi_thermal_adb_reboot(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.power import get_power_obj
        power = get_power_obj()
        self.thermal_file_host = gen_tmp_file_path_by_time("adb_reboot.txt")
        self.thermal_data_result = gen_tmp_file_path_by_time("adb_reboot.xls")
        write_to_file("cpu_temp board_temp cpu_freq power gpu_freq\n", self.thermal_file_host, "w")
        data_list = self.ivi_capture_thermal_data()
        write_to_file(data_list, self.thermal_file_host, "a")
        times = 10
        for i in range(1, 1 + times):
            print "Cycle: %d/%d" % (i, times)
            power.adb_reboot()
            self.thermal.adb_root()
            data_list = self.ivi_capture_thermal_data()
            write_to_file(data_list, self.thermal_file_host, "a")
        self.thermal.ivi_gen_temp_result(self.thermal_data_result, self.thermal_file_host, self.score_list)

