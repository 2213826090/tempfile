#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
#import re
from nohup_process import NohupProcess
from tools import *
from testlib.em.apps import Chrome

class Thermal(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.thermal_dir = "/sys/class/thermal"

    def ivi_start_capture_thermal_data(self, thermal_file):
        if self.get_product() == BXT_O:
            script_name = "ivi_get_thermal_o.sh"
        else:
            script_name = "ivi_get_thermal.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        self.np = NohupProcess(self.testDevice, script_path, "/data/local/tmp/", thermal_file)
        self.np.start()

    def ivi_end_capture_thermal_data(self):
        if hasattr(self, "np"):
            self.np.stop()

#    def ivi_check_thermal_data(self, data_file):
#        lines = self.read_file_by_line(data_file)
#        colums = len(lines.next().split())
#        #print colums
#        for line in lines:
#            line_list = line.split()
#            assert len(line_list) == colums
#            for item in line_list[1:]:
#                int(item)
#
    def ivi_gen_temp_result(self, result_file, data_file, score_list = []):
        import xlwt
        workbook = xlwt.Workbook(encoding = 'ascii')
        # temp sheet
        sheet_temp = workbook.add_sheet('temp')
        lines = read_file_by_line(data_file)
        col = 0
        for item in lines.next().split():
            sheet_temp.write(0, col, label = item)
            col += 1
        row = 1
        for line in lines:
            col = 0
            line_list = line.split()
            for item in line_list:
                sheet_temp.write(row, col, label = int(item))
                col += 1
            row += 1
        if score_list:
            # score sheet
            sheet_score = workbook.add_sheet('score')
            sheet_score.write(0, 0, label = "score")
            row = 1
            for score in score_list:
                sheet_score.write(row, 0, label = score)
                row += 1
        # save workbook
        workbook.save(result_file)

    def get_fan_status(self):
        from ioc_log import IOC
        ioc = IOC()
        ioc.start()
        board_temp, ambient_temp, fan_percent = ioc.get_fan_status()
        ioc.stop_reading()
        ioc.join()
        return board_temp, ambient_temp, fan_percent

    def map_fan_percent_m(self, cpu_temp, board_temp, ambient_temp, fan_percent):
        if cpu_temp < 60:
            cpu_percent = 0
        elif cpu_temp < 70:
            cpu_percent = 10
        elif cpu_temp < 80:
            cpu_percent = 20
        elif cpu_temp < 90:
            cpu_percent = 30
        elif cpu_temp < 100:
            cpu_percent = 50
        else:
            cpu_percent = 100

        if board_temp < 50:
            board_percent = 0
        elif board_temp < 60:
            board_percent = 40
        elif board_temp < 70:
            board_percent = 60
        else:
            board_percent = 100

        if ambient_temp < 40:
            ambient_percent = 0
        elif ambient_temp < 50:
            ambient_percent = 30
        elif ambient_temp < 60:
            ambient_percent = 50
        else:
            ambient_percent = 100

        return max(cpu_percent, board_percent, ambient_percent) == fan_percent

    def map_fan_percent_o(self, cpu_temp, board_temp, ambient_temp, fan_percent):
        temp = max(board_temp, ambient_temp)
        if temp < 30:
            return fan_percent == 0
        elif temp < 50:
            return 0 <= fan_percent <= 100
        else:
            return fan_percent == 100

    def map_fan_percent(self, cpu_temp, board_temp, ambient_temp, fan_percent):
        if self.get_product() == BXT_M:
            return self.map_fan_percent_m(cpu_temp, board_temp, ambient_temp, fan_percent)
        else:
            return self.map_fan_percent_o(cpu_temp, board_temp, ambient_temp, fan_percent)


class ThermalNormal(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.serial = self.d.server.adb.device_serial()
        self.thermal_dir = "/sys/class/thermal"

    def check_thermal_zone(self, num = 1):
        print "[info]--- Check thermal daemon zone%d" % num
        cmd = "cat %s/thermal_zone%d/" % (self.thermal_dir, num)
        msg = self.testDevice.adb_cmd_capture_msg(cmd + "temp")
        int(msg)

        msg = self.testDevice.adb_cmd_capture_msg(cmd + "type")
        assert msg != ""
        if num == 0:
            return
        if "cht" in self.get_product():
            return
        # zone 3 - 7
        if num > 2:
            msg = self.testDevice.adb_cmd_capture_msg(cmd + "policy")
            assert msg != ""
            msg = self.testDevice.adb_cmd_capture_msg("ls %s%d/ | grep passive" % (self.thermal_dir, num))
            if msg != "":
                msg = self.testDevice.adb_cmd_capture_msg(cmd + "policy")
                assert msg != ""
            return
        # zone 1 - 2
        msg = self.testDevice.adb_cmd_capture_msg(cmd + "trip_point_0_temp")
        int(msg)
        msg = self.testDevice.adb_cmd_capture_msg(cmd + "trip_point_0_type")
        assert msg != ""
        if not self.product in ["r0_bxtp_abl", "gordon_peak"]:
            msg = self.testDevice.adb_cmd_capture_msg(cmd + "trip_point_1_temp")
            int(msg)
            msg = self.testDevice.adb_cmd_capture_msg(cmd + "trip_point_1_type")
            assert msg != ""

    def check_thermal_zone_in_logcat(self, time_str = None):
        if time_str:
            logcat_cmd = "logcat -t '%s' -s ThermalZone" % time_str
        else:
            logcat_cmd = "logcat -d -s ThermalZone"
        cmd = logcat_cmd + " | grep UEvent | head -n 1"
        msg = self.testDevice.adb_cmd_common(cmd)
        if msg != "":
            print msg
            return True
        return False

    def push_hd_video_to_device(self, cfg, option_location, option_file):
        video_file = cfg.get(option_file).split('/')[-1]
        cmd = "ls /mnt/sdcard/Movies/ | grep %s" % video_file
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        if msg != "":
            print "[info]---video file %s already existed." % video_file
            return
        print "[info]--- Push %s to device." % video_file
        ret_file = self.download_content(cfg, option_location, option_file)
        assert os.path.isfile(ret_file)
        self.testDevice.adb_cmd("mkdir -p /mnt/sdcard/Movies/")
        self.testDevice.adb_cmd_common("push %s /mnt/sdcard/Movies/" % ret_file, 500)
        self.testDevice.adb_cmd("am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/sdcard/videos")

    def install_robocop_cache(self, cfg, option_location, option_file):
        cache_dir = "/mnt/sdcard/Android/obb/com.glu.robocop"
        cmd = "ls %s | grep com.glu.robocop.obb" % cache_dir
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        if msg != "":
            print "[info]---cache file %s already existed." % msg
            return
        print "[info]--- Install robocop cache"
        ret_file = self.download_content(cfg, option_location, option_file)
        assert os.path.isfile(ret_file)
        self.testDevice.adb_cmd("mkdir -p %s" % cache_dir)
        self.testDevice.adb_cmd_common("push %s %s" % (ret_file, cache_dir))

    def get_display_info(self):
        info = self.d.info
        print "[info]--- display:", info["displayWidth"], info["displayHeight"]
        return info["displayWidth"], info["displayHeight"]

    def launch_robocop(self):
        print "[info]--- Launch robocop"
        self.testDevice.launch_app_am("com.glu.robocop", "com.google.android.vending.expansion.downloader_impl.DownloaderActivity")

    def start_play_robocop(self, displayWidth, displayHeight):
        print "[info]--- Start play robocop"
        display_x = max(displayWidth, displayHeight)
        display_y = min(displayWidth, displayHeight)
        x = display_x * 7 / 10
        y = display_y * 5 / 8
        y2 = display_y * 7 / 8
        print (x, y)
        for i in range(5):
            time.sleep(10)
            self.d.click(x, y2)
            time.sleep(10)
            self.d.click(x, y)

    def get_zones_count(self):
        cmd = "ls %s/ | grep -c thermal_zone" % self.thermal_dir
        count = self.testDevice.adb_cmd_capture_msg(cmd)
        print "[info]--- zones count:", count
        return int(count)

    @property
    def zones_count(self):
        if not hasattr(self, "_zones_count"):
            self._zones_count = self.get_zones_count()
        return self._zones_count

    def get_zones_temps(self, zone_list = range(9)):
        zones_path = " thermal_zone%d/temp" * len(zone_list) % tuple(zone_list)
        cmd = "'cd %s; cat %s'" % (self.thermal_dir, zones_path)
        str_temps = self.testDevice.adb_cmd_capture_msg(cmd)
        return str_temps.splitlines()

    def check_temps_lower_than(self, temps, val):
        for temp in temps:
            assert temp < val

    def get_thermal_log_dir(self):
        if not hasattr(self, "thermal_log_dir"):
            base_name = "thermal_%s_%s" % (self.serial, time.strftime("%Y-%m-%d-%H-%M-%S"))
            self.thermal_log_dir = os.path.join(os.path.expanduser("~/.oat/logs"), base_name)
            os.makedirs(self.thermal_log_dir, mode = 0777)
            print "[info]--- save files to dir:", self.thermal_log_dir
        return self.thermal_log_dir
        #return "/home/yang/Desktop/temps/test_3GR"

    def start_thermal_capture_background(self):
        thermal_log = os.path.join(self.get_thermal_log_dir(), "thermal.txt")
        script = os.path.join(os.path.dirname(__file__), "thermal_capture.py")
        cmd = "python %s %s %s %s" % (script, self.serial, self.zones_count, thermal_log)
        self.sub_process = subprocess.Popen(cmd, shell = True)
        return thermal_log

    def terminate_subprocess(self):
        if hasattr(self, "sub_process"):
            from testlib.util.process import killall
            killall(self.sub_process.pid)
            delattr(self, "sub_process")

    def start_thermal_capture_on_device(self):
        script_file = os.path.join(os.path.dirname(__file__), "device_scripts", "thermal_capture.sh")
        self.testDevice.adb_cmd_common("push %s /data/local/tmp/" % script_file)
        self.testDevice.adb_cmd("'cd /data/local/tmp/; nohup sh thermal_capture.sh %d /sdcard/thermal.txt' &" % self.zones_count)

    def end_thermal_capture_on_device(self):
        process_msg = self.testDevice.adb_cmd_capture_msg("ps -C sh | grep root")
        for process in process_msg.splitlines():
            arr_info = process.split()
            if arr_info[-1] == "sh":
                self.testDevice.adb_cmd("kill " + arr_info[1])

    def pull_thermal_log_to_host(self):
        self.testDevice.adb_cmd_common("pull /sdcard/thermal.txt " + self.get_thermal_log_dir())

    def zone_temp_top_ten_average(self):
        thermal_log_file = os.path.join(self.get_thermal_log_dir(), "thermal.txt")
        zone_count = self.zones_count
        zone_temps_arr = []
        for i in range(zone_count):
            zone_temps_arr.append([])
        zone_temps_ave = [0] * zone_count
        for line in read_file_by_line(thermal_log_file):
            line_arr = line.split()
            if len(line_arr) > zone_count and line_arr[0] != "date":
                for i in range(zone_count):
                    zone_temps_arr[i].append(int(line_arr[i + 1]))
        sample_num = min(10, len(zone_temps_arr[0]))
        for i in range(zone_count):
            zone_temps_arr[i].sort(reverse = True)
            temp_sum = 0
            for j in range(sample_num):
                temp_sum += zone_temps_arr[i][j]
            zone_temps_ave[i] = temp_sum / sample_num
        print "[info]--- zone temps top 10 on average:", zone_temps_ave

    def generate_fig(self, table_score = []):
        thermal_log = os.path.join(self.get_thermal_log_dir(), "thermal.txt")
        thermal_fig = os.path.join(self.get_thermal_log_dir(), "fig.jpg")
        if os.path.isfile(thermal_log):
            cur_dir = os.path.dirname(__file__)
            print cur_dir
            gen_fig = os.path.join(cur_dir, "generate_figure.py")
            string_table = " %d" * len(table_score) % tuple(table_score)
            cmd_args = "%s %s %d %s" % (thermal_log, thermal_fig, self.zones_count, string_table)
            os.system("python %s %s" % (gen_fig, cmd_args))
        if os.path.isfile(thermal_fig):
            print "[info]--- generate figure:", thermal_fig
        else:
            print "[info]--- generate figure failed"

    def parse_thermal_sensor_config(self):
        self.testDevice.adb_cmd_common("pull /system/etc/thermal_sensor_config.xml " + self.get_thermal_log_dir())
        import xml.etree.cElementTree as ET
        thermal_config_file = os.path.join(self.get_thermal_log_dir(), "thermal_sensor_config.xml")
        tree = ET.parse(thermal_config_file)
        root = tree.getroot()
        zone_threshold_dict = {}
        def get_threshold_value(item_obj, tag, arr):
            child = item_obj.find(tag)
            if child != None:
                value = int(item_obj.find(tag).text)
                if value < 1000:
                    value *= 1000
                arr.append(value)
        for profile in root.findall("Profile"):
            if profile.find("Name").text == "Default":
                for zone in profile.findall("Zone"):
                    value_arr = []
                    threshold = zone.find("ZoneThreshold")
                    get_threshold_value(threshold, "ZoneThresholdNormal", value_arr)
                    get_threshold_value(threshold, "ZoneThresholdWarning", value_arr)
                    get_threshold_value(threshold, "ZoneThresholdWarning1", value_arr)
                    get_threshold_value(threshold, "ZoneThresholdWarning2", value_arr)
                    get_threshold_value(threshold, "ZoneThresholdAlert", value_arr)
                    for sensor_attr in zone.findall("SensorAttrib"):
                        key = sensor_attr.find("SensorName").text
                        zone_threshold_dict[key] = value_arr
                break
        return zone_threshold_dict

    def parse_thermal_throttle_config(self):
        self.testDevice.adb_cmd_common("pull /system/etc/thermal_throttle_config.xml " + self.get_thermal_log_dir())
        import xml.etree.cElementTree as ET
        thermal_config_file = os.path.join(self.get_thermal_log_dir(), "thermal_throttle_config.xml")
        tree = ET.parse(thermal_config_file)
        root = tree.getroot()
        throttle_dict = {}
        def get_throttle_value(item_obj, tag, arr):
            child = item_obj.find(tag)
            if child != None:
                value = int(item_obj.find(tag).text)
                arr.append(value)
        for CDevice in root.findall("ContributingDeviceInfo"):
            text = CDevice.find("CDeviceName").text
            if text in ["CPU", "CPUfreq", "GPU", "Display"]:
                value_arr = []
                throttle = CDevice.find("ThrottleValues")
                get_throttle_value(throttle, "ThrottleNormal", value_arr)
                get_throttle_value(throttle, "ThrottleWarning", value_arr)
                get_throttle_value(throttle, "ThrottleWarning1", value_arr)
                get_throttle_value(throttle, "ThrottleWarning2", value_arr)
                get_throttle_value(throttle, "ThrottleAlert", value_arr)
                throttle_dict[text] = value_arr
        return throttle_dict

    def read_thermal_log_to_dict(self):
        thermal_log = os.path.join(self.get_thermal_log_dir(), "thermal.txt")
        line_iter = read_file_by_line(thermal_log)
        title_arr = line_iter.next().split()
        thermal_dict = {}
        for title in title_arr:
            thermal_dict[title] = []
        for line in line_iter:
            line_arr = line.split()
            for i in range(len(title_arr)):
                thermal_dict[title_arr[i]].append(line_arr[i])
        return title_arr, thermal_dict

    def get_threshold_index(self, threshold_arr, temp):
        j = 0
        for threshold in threshold_arr:
            if temp < threshold:
                break
            j += 1
        return j

    # check common throttle, such as CPU, GPU, LCD
    def check_throttle_by_zone(self, zone_name, temp_arr, freq_arr, thermal_threshold_arr, freq_throttle_arr):
        n = 1
        for str_temp, str_freq in zip(temp_arr, freq_arr):
            n += 1
            temp = int(str_temp)
            upper_limit = thermal_threshold_arr[-1] - 10*1000
            assert temp <= upper_limit, "line %d, %s , temp higher than %d" % (n, zone_name, upper_limit)
            index = self.get_threshold_index(thermal_threshold_arr, temp)
            assert int(str_freq) <= freq_throttle_arr[index], "line %d, %s, freq too high" % (n, zone_name)

    def check_battery_throttle(self, temp_arr, charge_status_arr, battery_threshold):
        ignore_head_tail = 0
        for i in range(ignore_head_tail, len(charge_status_arr) - ignore_head_tail):
            temp = int(temp_arr[i])
            real_status = charge_status_arr[i]
            right_status = "yes" if temp < battery_threshold else "no"
            assert real_status == right_status, "line %d, charging status wrong" % (i + 2)

    def check_other_zone_threshold(self, zone_name, temp_arr, threshold_alert):
        n = 1
        upper_limit = threshold_alert - 10*1000
        for temp in temp_arr:
            n += 1
            assert int(temp) <= upper_limit, "line %d, %s , temp higher than %d" % (n, zone_name, upper_limit)

    def check_zone_temp_and_throttle(self, threshold_dict, throttle_dict, check_charging = False):
        key_arr, thermal_dict = self.read_thermal_log_to_dict()
        # check CPU zone: coretemp0~3
        if throttle_dict.has_key("CPUfreq"):
            freq_throttle_arr = throttle_dict["CPUfreq"]
        else:
            freq_throttle_arr = throttle_dict["CPU"]
        for cpu_id in range(4):
            zone_name = "coretemp%d" % cpu_id
            if thermal_dict.has_key(zone_name):
                cpu_temp_arr = thermal_dict[zone_name]
                cpu_freq_arr = thermal_dict["cpu%d_cur" % cpu_id]
                thermal_threshold_arr = threshold_dict[zone_name]
                self.check_throttle_by_zone(zone_name, cpu_temp_arr, cpu_freq_arr, thermal_threshold_arr, freq_throttle_arr)
        # check GPU zone: GPU(LTE) or coretemp5(3GR)
        if threshold_dict.has_key("GPU"):
            zone_name = "GPU"
        else:
            zone_name = "coretemp5"
        freq_throttle_arr = throttle_dict["GPU"]
        gpu_temp_arr = thermal_dict[zone_name]
        gpu_freq_arr = thermal_dict["cpu0_gpu"]
        thermal_threshold_arr = threshold_dict[zone_name]
        self.check_throttle_by_zone(zone_name, gpu_temp_arr, gpu_freq_arr, thermal_threshold_arr, freq_throttle_arr)
        # check battery zone
        if check_charging:
            if threshold_dict.has_key("PMIC_BPTHERM0_Battery"):
                battery_threshold = threshold_dict["PMIC_BPTHERM0_Battery"][-1]
            else:
                battery_threshold = threshold_dict["battery"][-1]
            battery_temp_arr = thermal_dict["battery"]
            charging_status_arr = thermal_dict["charging"]
            self.check_battery_throttle(battery_temp_arr, charging_status_arr, battery_threshold)
        # check other zones
        to_remove = ["coretemp%d" % i for i in range(6)] + ["GPU", "battery"]
        for zone_id in range(self.zones_count):
            zone_name = key_arr[zone_id]
            if zone_name not in to_remove and threshold_dict.has_key(zone_name):
                threshold_alert = threshold_dict[zone_name][-1]
                temp_arr = thermal_dict[zone_name]
                self.check_other_zone_threshold(zone_name, temp_arr, threshold_alert)

