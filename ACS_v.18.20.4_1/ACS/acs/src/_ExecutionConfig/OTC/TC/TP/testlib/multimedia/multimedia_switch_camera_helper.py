# coding: UTF-8
import subprocess

from testlib.multimedia.get_device_port_helper import GetConfigFileHelper
from testlib.androidframework.common import SystemUtils
from testlib.androidframework.adb_utils import AdbUtils

from testlib.util.log import Logger
logger = Logger.getlogger()


class MultiMediaSwitchCameraHelper:

    def __init__(self, cfg_file="", skip_import_camera=False):
        self.cfg_file = GetConfigFileHelper(cfg_file, "multimedia_switch_camera_helper.conf")

        self.android_version = self.get_android_version().lower()
        self.platform_name = SystemUtils.get_platform_name()

        self.camera = ""
        self.switchplatform(skip_import_camera)

    def __execute_command_with_popen(self, cmd): #fix bug
        logger.debug("__execute_command_with_popen cmd=%s" % cmd)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    def get_android_version(self):
        cmd = AdbUtils.prepare_cmd('getprop | grep ro.build.version.sdk', True)
        sdk_string = self.__execute_command_with_popen(cmd).stdout.read()
        if '26' in sdk_string or '27' in sdk_string:
            return "O"
        elif '24' in sdk_string or '25' in sdk_string:
            return "N"
        elif '23' in sdk_string:
            return "M"
        elif '22' in sdk_string:
            return "L"

    def switchplatform(self, skip_import_camera=False):
        support_devices = self.cfg_file.get_section("support_devices")
        self.device = ""
        for t_device in support_devices.keys():
            if self.platform_name in support_devices[t_device]:
                self.device = t_device
                break
        assert self.device != "", "can't support this devices! platform_name:%s" % self.platform_name
        logger.debug(self.cfg_file.get_section("device_camera"))
        if self.android_version == "n":
            cmd = 'getprop ro.build.version.release'
            sdk_string = AdbUtils.run_adb_cmd(cmd).strip().lower()
            logger.debug("sdk_string : %s" % sdk_string)
            if "o" in sdk_string:
                self.android_version = "o"
        camera_name = self.cfg_file.get_section("device_camera")["%s---%s" % (self.device, self.android_version)]
        if not skip_import_camera:
            import_string = "testlib.camera.%s" % (camera_name)
            if import_string == "":
                assert 0, "import_string error!"
            else:
                logger.debug("set %s camera" % camera_name)
                module = __import__(import_string, globals(), locals(), ['json', 'title'], -1)
    #             module = __import__(import_string)
    #             import sys
    #             module = sys.modules[import_string]
                logger.debug("module:%s" % module)
                camera_class = getattr(module, camera_name)
                self.camera = camera_class()
            return self.camera
