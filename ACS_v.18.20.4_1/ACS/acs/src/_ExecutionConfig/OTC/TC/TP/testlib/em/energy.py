#` -*- coding: utf-8 -*-
import os
import sys
import time
import re
import random

from nohup_process import NohupProcess
#from constants_def import *
from tools import *
from usb_cut import USBCut


class Energy(UIBase):
    """
    The Energy class is the base library for test cases regarding to Energy Management.
    """

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)

    #def get_sdp_charge_status(self):
    #    cmd = "dumpsys battery | grep 'USB powered: true'"
    #    result = self.testDevice.adb_cmd_capture_msg(cmd)
    #    if result:
    #        return True
    #    else:
    #        return False

    def get_sys_battery_dir(self):
        if not hasattr(self, "_sys_battery_dir"):
            power_supply_dir = "/sys/class/power_supply/"
            msg = self.testDevice.adb_cmd_capture_msg("ls %s" % power_supply_dir)
            dirs = msg.split()
            for battery_dir in ["dollar_cove_battery", "intel_fuel_gauge", "max170xx_battery", "battery"]:
                if battery_dir in dirs:
                    break
            else:
                assert False, "sys battery dir not found"
            self._sys_battery_dir = os.path.join(power_supply_dir, battery_dir)
        return self._sys_battery_dir

    def get_battery_info(self):
        cmd = "dumpsys battery"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        if not msg:
            return None
        elif "error:" in msg:
            return None
        msg_dict = {}
        ret_dict = {}
        for line in msg.splitlines():
            line_arr = line.split(": ")
            if len(line_arr) < 2:
                continue
            msg_dict[line_arr[0].strip()] = line_arr[1].strip()
        ret_dict["AC"] = msg_dict["AC powered"] == "true"
        ret_dict["USB"] = msg_dict["USB powered"] == "true"
        ret_dict["temp"] = int(msg_dict["temperature"]) / 10
        ret_dict["voltage"] = int(msg_dict["voltage"])
        ret_dict["level"] = int(msg_dict["level"])
        return ret_dict

    def capture_screen_not_charging(self, screenshot):
        script_name = "capture_screen.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        exec_dir = "/mnt/sdcard/"
        screenshot_on_device = "/mnt/sdcard/screenshot.png"
        np = NohupProcess(self.testDevice, script_path, exec_dir, "5 %s" % screenshot_on_device)
        np.start()
        USBCut().cut(10)
        cmd = "pull %s %s" % (screenshot_on_device, screenshot)
        self.testDevice.adb_cmd_common(cmd)

    def set_virtual_battery_level(self, percent = 0):
        cmd = "am broadcast -a android.intent.action.BATTERY_CHANGED --ei level %d --ei scale 100" % percent
        self.testDevice.adb_cmd(cmd)

    def get_actual_backlight(self):
        msg = self.testDevice.adb_cmd_capture_msg("dumpsys display | grep mActualBacklight=")
        backlight = msg.split("=")[1]
        print "[info]--- actual backlight:", backlight
        return int(backlight)

    def get_actual_backlight_battery_saver(self):
        from settings import BatterySaverSetting
        script_name = "battery_saver.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        exec_dir = "/mnt/sdcard/"
        BatterySaverSetting().launch()
        bounds = self.d(className="android.widget.Switch").bounds
        x = (bounds["left"] + bounds["right"]) / 2
        y = (bounds["top"] + bounds["bottom"]) / 2
        display_info_txt = "/mnt/sdcard/display.txt"
        string_args = "5 %s %s %s" % (x, y, display_info_txt)
        np = NohupProcess(self.testDevice, script_path, exec_dir, string_args)
        np.start()
        USBCut().cut(10)
        cmd = "cat %s | grep mActualBacklight=" % display_info_txt
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        backlight = msg.split("=")[1]
        print "[info]--- actual backlight:", backlight
        return int(backlight)

    def get_charger_type_path(self):
        cmd = "ls /sys/class/power_supply/*charger -d"
        msg_list = self.testDevice.adb_cmd_capture_msg(cmd).split()
        if len(msg_list) == 1:
            return msg_list[0] + "/type"
        return None

    def start_copy_file_service(self, source_file, target_file, interval = 0.5):
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", "copy_charger_type.sh")
        string_args = "%s %s %s" % (source_file, target_file, interval)
        self.np = NohupProcess(self.testDevice, script_path, "/data/local/tmp/", string_args)
        self.np.start()

    def end_copy_file_service(self):
        self.np.stop()

    def start_monitor_screen_on_off(self):
        self.testDevice.launch_app_am(EM_TOOLS_PACKAGE, EM_TOOLS_SCREEN_ACTIVITY)
        time.sleep(2)

    def clean_screen_on_off_history(self):
        self.d(resourceId="com.intel.yang.emtools:id/clean_screen_on_off_history").click()

    def get_screen_on_off_history(self):
        return self.d(resourceId="com.intel.yang.emtools:id/screen_on_off_history").text

    def read_uidump_xml(self, uidump_file):
        import xml.etree.cElementTree as ET
        tree = ET.parse(uidump_file)
        root = tree.getroot()
        return root

    def find_ui_view(self, xml_element_root, prop, value, result):
        """
        result: [], arr to store the results
        """
        for node in xml_element_root.findall("node"):
            if node.get(prop) == value:
                result.append(node)
            self.find_ui_view(node, prop, value, result)

    def get_battery_status_from_uidump(self, uidump_file):
        root = self.read_uidump_xml(uidump_file)
        result = []
        self.find_ui_view(root, "resource-id", "com.android.settings:id/estimation", result)
        if len(result) == 1:
            status = result[0].get("text")
            print status
            return status
        result = []
        self.find_ui_view(root, "resource-id", "com.android.settings:id/battery_history_chart", result)
        if len(result) == 1:
            status = result[0].get("content-desc")
            print status
            return status

    def check_sdp_charging_in_settings(self):
        if self.d(resourceId = "com.android.settings:id/battery_history_chart").exists:
            text = self.d(resourceId = "com.android.settings:id/battery_history_chart").description
            status = text.split("%")[1]
        else:
            status = self.d(resourceId = "com.android.settings:id/estimation").text
        print "[info]--- Battery status: %s" % status
        if "USB" in status:
            return True
        return False

    def capture_uidump_with_charging(self, uidump_file, charger_type = None):
        from constants_def import CDP, DCP
        script_name = "dump_ui.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        exec_dir = "/mnt/sdcard/"
        uidump_on_device = "/mnt/sdcard/uidump.xml"
        np = NohupProcess(self.testDevice, script_path, exec_dir, "6 %s" % uidump_on_device)
        np.start()
        if charger_type == None:
            USBCut().cut(10)
        elif charger_type in [CDP, DCP]:
            if charger_type == CDP:
                self.enable_cdp_charging(1, 0)
            else:
                self.enable_dcp_charging(1, 0)
            time.sleep(10)
            self.three_way_cutter_reconnect_sdp(3, 2, 5)
        cmd = "pull %s %s" % (uidump_on_device, uidump_file)
        self.testDevice.adb_cmd_common(cmd)

    def check_charging_full(self):
        text = self.d(resourceId = "com.android.settings:id/battery_history_chart").description
        print "[info]--- Charging status:", text
        level, status = text.split("%")
        assert level == "100", "percentage not 100"
        assert "- Full" in status, "not full"

    def charge_to_percentage(self, target):
        cycles = 20
        for i in range(1, 1 + cycles):
            level = self.get_battery_level()
            if level >= target:
                break
            wait_time = (target - level + 1) * 60
            print "[RunTest]: Charge cycle: %s/%s, wait %ss" % (i, cycles, wait_time)
            self.enable_dcp_charging()
            time.sleep(wait_time)
            self.three_way_cutter_reconnect_sdp(3, 2, 5)
        else:
            assert False

