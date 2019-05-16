# coding: UTF-8
'''
Created on May 04, 2018

@author: Li Zixi
'''
import time
from testlib.graphics.common import adb32
from testlib.util.common import g_common_obj
from testlib.camera.CameraCommon import CameraCommon
from testlib.multimedia.get_device_port_helper import GetConfigFileHelper
from testlib.multimedia.multimedia_setting import MultiMediaSetting

from testlib.util.log import Logger
logger = Logger.getlogger()

class MulticameraApkHelper:

    MULTI_CAMERA_CONFIG_DICT = {"camera0_input_field" : 1
                                ,"camera0_input_resolution" :3
                                ,"camera0_input_format" : 4
                                ,"camera0_input_default" : 5
                                ,"camera0_output_resolution" : 0
                                ,"camera0_output_format" : 2
                                ,"camera1_input_field" : 7
                                ,"camera1_input_resolution" :9
                                ,"camera1_input_format" : 10
                                ,"camera1_input_default" : 11
                                ,"camera1_output_resolution" : 6
                                ,"camera1_output_format" : 8}

    def __init__(self):
        self.d = g_common_obj.get_device()
        self.camera_common = CameraCommon()
        self.get_cfg_file_helper = GetConfigFileHelper("", "multicamera_apk_helper.conf")
        self.multimedia_setting = MultiMediaSetting(self.get_cfg_file_helper.cfg_file)
        need_install_result = self.multimedia_setting.install_apk("multicamera_apk")
        if need_install_result:
            adb32._adb_reboot()
            time.sleep(15)
        self.multicamera_apk_package_name, _ = self.multimedia_setting.get_package_and_activity_name("multicamera_apk")
        MULTI_CAMERA_PERMISSION_LIST = ["android.permission.CAMERA"]
        self.camera_common.grantPermission(self.multicamera_apk_package_name, MULTI_CAMERA_PERMISSION_LIST)

    def launch_multicamera_apk(self):
        return self.multimedia_setting.launch_apk("multicamera_apk")

    def stop_multicamera_apk(self):
        return self.multimedia_setting.stop_apk("multicamera_apk")

    def set_multicamera_config(self, config_list):
        logger.debug("set_multicamera_config start. config_list=%s" % config_list)
        for config_str in config_list:
            config_item, config_value = config_str.split(":")
            if config_item not in self.MULTI_CAMERA_CONFIG_DICT.keys():
                continue
            logger.debug("config_item:%s, config_value:%s" % (config_item, config_value))
            self.d(className="android.widget.Spinner")[self.MULTI_CAMERA_CONFIG_DICT[config_item]].click.wait()
            time.sleep(1)
            self.d(textContains=config_value).click.wait()
            time.sleep(1)
        logger.debug("set_multicamera_config end.")