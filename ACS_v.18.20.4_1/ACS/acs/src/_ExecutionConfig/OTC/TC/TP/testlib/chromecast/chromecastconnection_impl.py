# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for ChromeCast operation
@since: 02/25/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl


class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

class ChromeCastImpl:
    '''
    classdocs
    '''
    pkg_name = "com.android.settings"
    activity_name = "com.android.settings.Settings"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def set_environment(self):
        """ init the test environment
        """
        self.systemui = SystemUiExtendImpl()
        self.camera = CameraExtendImpl()
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_cast = config.read(cfg_file, 'chromecast_device')
        self.nexus_device = cfg_cast.get("device")
        self.nexus_name = cfg_cast.get("name")
        cmd = "-s %s logcat -d -c" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        print "[clear log]"

    def launch_app_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(\
            ChromeCastImpl.pkg_name, ChromeCastImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am(ChromeCastImpl.pkg_name)

    def goto_castscreen(self):
        """ go to the castscreen interface
        """
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Cast").click()
        time.sleep(5)

    def scan_adapter(self):
        """ scan the ChromeCast adapter
        """
        time.sleep(5)
        if self._device(text="No nearby devices were found.").exists:
            time.sleep(30)
        assert self._device(className="android.widget.RelativeLayout"
            ).exists , "No nearby chromecast adapter"
        assert self._device(text=self.nexus_name).exists, "No target chromecaset exists."

    def connect_chromecast(self):
        """ connect to the chromecast
        """
        self._device(text=self.nexus_name).click()
        time.sleep(5)
        cmd = "-s %s logcat -d > /tmp/connect.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/connect.log|grep 'onCastMirroringConnected'"\
            ), "The chromecast didn't connect "

    def disconnect_chromecast(self):
        """ disconnect the chromecast
        """
        self._device(text=self.nexus_name).click()
        time.sleep(2)
        self._device(text="Disconnect").click()
        time.sleep(2)
        cmd = "-s %s logcat -d > /tmp/disconnect.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/disconnect.log|grep 'onCastMirroringConnected'"\
            ), "The chromecast didn't disconnect "

    def reconnect_chromecastx10(self):
        """ replay connect to chromecast 10 times
        """
        for i in range(1, 11):
            self.scan_adapter()
            self.connect_chromecast()
            time.sleep(15)
            self.swith_to_camera_and_return()
            self.disconnect_chromecast()
            print  "[reconnect to chromecast %s times]" % i
            time.sleep(2)

    @staticmethod
    def reboot_device():
        """ reboot the device
        """
        g_common_obj.reboot_device()
        time.sleep(40)

    def reboot_chromecast_adapter(self):
        """ reboot the chromecast adapter
        """
        cmd = "-s %s reboot" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        print "[Wait adapter reboot]"
        time.sleep(100)

    def swith_to_camera_and_return(self):
        """ Launch camera via adb am command
        """
        print "Launch camera by adb am"
        self.camera.launch_camera_am()
        self._locator.wait_exist(self._locator.performance_tests)
        if self._device(text="NEXT").exists:
            self._device(text="NEXT").click()
        self.systemui.switch_recent_app("Settings")

    def disconnect_cast_if_failed(self):
        """ disconnect chromecast if failed
        """
        self.launch_app_am()
        self.goto_castscreen()
        if self._device(text="Connected").exists:
            self.disconnect_chromecast()
        self.stop_app_am()
