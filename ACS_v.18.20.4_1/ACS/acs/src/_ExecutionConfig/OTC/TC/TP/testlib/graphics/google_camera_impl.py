# -*- coding: utf-8 -*-
'''
Created on Dec 5, 2014

@author: yusux
'''
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj


class Locator:

    def __init__(self, device):
        self._device = g_common_obj.get_device()
        self._jsonrpc = device.server.jsonrpc


class GoogleCameraImpl():
    pkg_name = "com.google.android.GoogleCamera"
    activity_name = "com.android.camera.CameraLauncher"

    def __init__(self):
        self._device = g_common_obj.get_device()

    def launch_app_am(self):
        print "Launch GoogleCamera by adb am"
        g_common_obj.launch_app_am(self.pkg_name,self.activity_name)

    def switch_back_home_serveraltimes(self, switch_times=20):
        SystemUI().unlock_screen()
        for x in range(switch_times):
            self.launch_app_am()
            self._device.screen.on()
            self._device.press("menu")
            self._device.press.home()
            self._device.wait.idle()
#             self.switch_back_home_serveraltimes()




