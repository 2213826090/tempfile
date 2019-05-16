# coding: UTF-8
import os
import subprocess
import re
import types
import time
from testlib.util.config import TestConfig
from testaid.usbrly import UsbRelay
from testaid.scroll import Scroll
from testaid.robot import Robot

from testlib.util.log import Logger
logger = Logger.getlogger()

def real_file_dir():
    return os.path.split(os.path.realpath(__file__))[0]

config = TestConfig()
cfg_file = os.path.join(real_file_dir(), "multimedia_lightbox_helper.conf")
cfg = config.read(cfg_file,  "config")



def executeCommandWithPopen(cmd):
    logger.debug("cmd=%s" % cmd)
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()

def getDevicePort(device_port_info, device_port_type="ttyUSB*"):
    cmd = "lsusb"
    t_usb_list = executeCommandWithPopen(cmd)[0][:-1]
    assert t_usb_list != "", "%s device can't find in lsusb" % device_port_info

    cmd = "ls -l /dev/%s" % (device_port_type)
    devices = executeCommandWithPopen(cmd)[0][:-1].split('\n')
    assert "No such file or directory" not in devices[0], "can't find %s device_port!" % device_port_info
    devices_dict = {}
    for device in devices:
#         device = " ".join(device.split())
        device_port = device.split(" ")[-1]
        if "dev" not in device_port:
            device_port = os.path.join("/dev", device_port)
        device_port_name = device_port.split('/')[-1]
        cmd = "ls -l /sys/class/tty/" + device_port_name
        t_output = executeCommandWithPopen(cmd)
        t_folder = "/sys/class/tty/" + t_output[0][:-1].split('->')[-1].strip(" ")
        t_idVendor = ""
        t_idProduct = ""
        t_port_info = ""
        for _ in range(5):
            idVendor_path = os.path.join(t_folder, "idVendor")
            if os.path.exists(idVendor_path):
                cmd = "cat %s" % idVendor_path
                t_idVendor = executeCommandWithPopen(cmd)[0][:-1]
                cmd = "cat %s" % os.path.join(t_folder, "idProduct")
                t_idProduct = executeCommandWithPopen(cmd)[0][:-1]
                get_device_port_info_parttern = re.compile(r"%s:%s (.*)[$\n]" % (t_idVendor, t_idProduct))
                t_port_info = get_device_port_info_parttern.findall(t_usb_list)[0]
                if device_port_info in t_port_info:
                    return device_port
                break
            t_folder = os.path.join(t_folder, "..")
        if t_idVendor == "":
            logger.debug("%s device can't find t_idVendor!" % device)
        devices_dict[device_port] = (t_idVendor, t_idProduct, t_port_info)
    logger.debug("devices_dict=%s" % devices_dict)
#     for device_port in devices_dict.keys():
#         if device_port_info in devices_dict[device_port][2]:
#             return device_port
    assert 0, "Can't find device_port in devices_dict: %s" % devices_dict

def initDevicePort(device_port_os_environ, device_port_config_name):
    if device_port_os_environ in os.environ:
        device_port = os.environ[device_port_os_environ]
    else:
        device_port = cfg.get(device_port_config_name)
        if device_port == "" or device_port == None:
            device_port_type = cfg.get("%s_type" % device_port_config_name)
            device_port_info = cfg.get("%s_info" % device_port_config_name)
            device_port = getDevicePort(device_port_info, device_port_type)
        os.environ["device_port_os_environ"] = device_port
    logger.debug("%s = %s" % (device_port_os_environ, device_port))
    return device_port


class MultiMediaLightBoxHelper:
    def __init__(self):
        device_port = initDevicePort("multimedia_lightbox_device_port", "light_box_device_port")
        self.light_box = UsbRelay(device_port)

        self.light_box_button_dict = eval(str(config.read(cfg_file,  "light_box_button_config")).lower())
        logger.debug(self.light_box_button_dict)
        self.power_button_index = self.light_box_button_index_convert("Power")
        self.light_status_dict = {self.power_button_index:0}

    def light_box_button_index_convert(self, index):
        logger.debug("light_box_button_index_convert: index=%s" % index)
        if type(index) is types.StringType:
            index = index.lower()
            if index in self.light_box_button_dict.keys():
                index = self.light_box_button_dict[index]
            index = int(index)
        return index

    def press_lightbox_button(self, index):
        index = self.light_box_button_index_convert(index)
        if index != self.power_button_index and self.light_status_dict[self.power_button_index] == 0:
            logger.debug("power button no press, skip press index(%s) button" % index)
            return -1
        logger.debug("press lightbox index(%s) button" % index)
        self.light_box.turn_on(index)
        self.light_box.turn_off(index)
        if index not in self.light_status_dict.keys() or self.light_status_dict[index] == 0:
            self.light_status_dict[index] = 1
        else:
            self.light_status_dict[index] = 0

    def press_light_with_list(self, index_list):
        for index in index_list:
            self.press_lightbox_button(index)
            time.sleep(1)

    def turn_off_all_light(self):
        if self.light_status_dict[self.power_button_index] == 1:
            self.press_lightbox_button(self.power_button_index)
            self.light_status_dict = {self.power_button_index:0}


class MultiMediaScrollHelper:
    def __init__(self):
        device_port = initDevicePort("multimedia_scroll_device_port", "scroll_device_port")
        self.scroll_device = Scroll(device_port)

    def reset(self):
        for i in range(3):
            logger.debug("MultiMediaScrollHelper().reset() %d time" % (i+1))
            try:
                self.scroll_device.reset()
                break
            except Exception as e:
                assert "JSON" in str(e), "MultiMediaScrollHelper().reset() error! %s" % e
                time.sleep(5)
        self.goto_page(0)

    def goto_page(self, num):
        return self.scroll_device.goto_page_2(num)

    def move(self, s1_displacement, s2_displacement, speed=6000):
        return self.scroll_device.move_2(s1_displacement, s2_displacement, speed)

    def move_to(self, s1_new_position, s2_new_position, speed=6000):
        return self.scroll_device.move_to_2(s1_new_position, s2_new_position, speed)

    def get_position(self):
        return self.scroll_device.get_position_2()

    def set_position(self, s1_new_position=0, s2_new_position=0):
        return self.scroll_device.set_position_2(s1_new_position, s2_new_position)

class MultiMediaRobotHelper:
    def __init__(self):
        device_port = initDevicePort("multimedia_robot_device_port", "robot_device_port")
        self.robot_device = Robot(device_port)
        self.default_position = int(cfg.get("robot_device_default_position"))
        self.default_angle = int(cfg.get("robot_device_default_angle"))

    def reset(self):
        for i in range(3):
            logger.debug("MultiMediaRobotHelper().reset() %d time" % (i+1))
            try:
                self.robot_device.reset()
                break
            except Exception as e:
                assert "JSON" in str(e), "MultiMediaRobotHelper().reset() error! %s" % e
                time.sleep(5)

    def move(self, length, speed):
        return self.robot_device.move(length, speed)

    def move_to(self, position, speed=100):
        return self.robot_device.move_to(position, speed)

    def rotate(self, angle, speed=40):
        return self.robot_device.rotate(angle, speed)

    def rotate_to(self, angle, speed=40):
        return self.robot_device.rotate_to(angle, speed)

    def swing(self, angle, speed=40):
        return self.robot_device.swing(angle, speed)

    def swing_to(self, angle, speed=40):
        return self.robot_device.swing_to(angle, speed)

    def move_to_default_position(self, position=-1, angle=-1):
        logger.debug("MultiMediaRobotHelper().move_to_default_position start")
        self.reset()
        if position == -1:
            position = self.default_position
        self.move_to(position)
        if angle == -1:
            angle = self.default_angle
        self.swing_to(angle)
        logger.debug("MultiMediaRobotHelper().move_to_default_position end")
