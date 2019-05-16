#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import sys
from func import OEMFunc
from tools import DeviceHandle
import os
from testlib.util.common import g_common_obj
from testlib.util.device import TestDevice
from testlib.util.repo import Artifactory

class oem_init():
    def __init__(self, sn):
        self.serial=sn
        self.func = OEMFunc(self.serial)
    def run(self):
        self.func.download_oem_content_artifactory('/IRDA_OEM_Customization/', 'OEM_file.zip')
        g_common_obj.root_on_device()
        self.func.mount_device()
        self.func.remove_folder()
        self.func.make_folder()
        self.func.push_oem_file()
        self.func.push_prop()
        self.func.setup_connection()
        self.func.unlock_screen()
        self.func.factory_reset()
        self.func.check_reset()
        self.func.wait_for_android_os()
        g_common_obj.root_on_device()
        self.func.mount_device()
        #self.func.setup_connection()
        g_common_obj.set_vertical_screen()
        self.func.skip_initial_screen_after_factory_reset()
        #self.func.keep_awake()
        #self.func.close_lock_screen()

    def reset_device_wall(self):
        g_common_obj.root_on_device()
        self.func.mount_device()
        self.func.reset_wallpaper()

