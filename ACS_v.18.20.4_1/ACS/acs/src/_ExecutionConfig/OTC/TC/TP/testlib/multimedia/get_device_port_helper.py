# coding: UTF-8
import os
import subprocess
import re
from testlib.util.config import TestConfig

from testlib.util.log import Logger
logger = Logger.getlogger()

def real_file_dir():
    return os.path.split(os.path.realpath(__file__))[0]

def executeCommandWithPopen(cmd, log_level=0):
    if log_level > 0:
        logger.debug("cmd=%s" % cmd)
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()

class GetConfigFileHelper:
    def __init__(self, config_dir, config_file):
        if config_dir == "":
            config_dir = real_file_dir()
        self.config = TestConfig()
        self.cfg_file = os.path.join(config_dir, config_file)

    def get_section(self, section="config"):
        return self.config.read(self.cfg_file, section)

class GetDevicePortHelper:
    def __init__(self, config_dir, config_file):
        self.get_config_file_helper = GetConfigFileHelper(config_dir, config_file)

    def get_device_port(self, device_port_info, device_port_type="ttyUSB*"):
        cmd = "lsusb"
        t_usb_list = executeCommandWithPopen(cmd)[0][:-1]
        assert t_usb_list != "", "%s device can't find in lsusb" % device_port_info

        device_port_type_list = [device_port_type]
        if device_port_type == "ttyCDI0":
            device_port_type_list.append("tty*")
        for t_device_port_type in device_port_type_list:
            cmd = "ls -l /dev/%s" % (t_device_port_type)
            devices = executeCommandWithPopen(cmd)[0][:-1].split('\n')
            if "No such file or directory" not in devices[0]:
                break
        assert "No such file or directory" not in devices[0], "Can't find devices in command '%s'!" % cmd
        if '*' not in t_device_port_type:
            logger.debug("skip search, use the config file. device_port_type=%s" % t_device_port_type)
            return "/dev/%s" % (t_device_port_type)
        devices_dict = {}
        for device in devices:
#             device = " ".join(device.split())
            device_port = device.split(" ")[-1]
            device_port_name = device_port.split('/')[-1]
#             print device_port
            if "relayCard" in device_port:
                logger.debug("skip search, use device_port=%s" % device_port)
                return device_port
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
#             if t_idVendor == "":
#                 logger.debug("%s device can't find t_idVendor!" % device)
            devices_dict[device_port] = (t_idVendor, t_idProduct, t_port_info)
        logger.debug("devices_dict=%s" % devices_dict)
#         for device_port in devices_dict.keys():
#             if device_port_info in devices_dict[device_port][2]:
#                 return device_port
        assert 0, "Can't find device_port in devices_dict: %s" % devices_dict

    def init_device_port(self, device_port_os_environ, device_port_config_name, section):
        if device_port_os_environ in os.environ:
            device_port = os.environ[device_port_os_environ]
        else:
            cfg = self.get_config_file_helper.get_section(section)
            device_port = cfg.get(device_port_config_name)
            if device_port == "" or device_port == None:
                device_port_type = cfg.get("%s_type" % device_port_config_name)
                device_port_info = cfg.get("%s_info" % device_port_config_name)
                device_port = self.get_device_port(device_port_info, device_port_type)
            os.environ["device_port_os_environ"] = device_port
        logger.debug("%s = %s" % (device_port_os_environ, device_port))
        return device_port
