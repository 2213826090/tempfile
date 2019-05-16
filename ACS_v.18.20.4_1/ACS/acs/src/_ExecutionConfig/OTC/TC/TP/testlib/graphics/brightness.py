# Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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

"""
@summary: This file implements for brightness
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj
import testlib.graphics.common as common
from testlib.graphics.common import dbsetting
from testlib.graphics.common import windows_info

class BrightnessImpl:
    """
    Battery Test Impl Class
    """
    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()

    def change_adaptive_brightness_status(self, status):
        if status.upper() == "ON":
            cmd = "settings put system screen_brightness_mode 1"
            g_common_obj.adb_cmd(cmd)
        elif status.upper() == "OFF":
            cmd = "settings put system screen_brightness_mode 0"
            g_common_obj.adb_cmd(cmd)

    def test_brightness_suspend_resume(self):
        brightness = [0, 255]
        for i in range(0, len(brightness)):
            dbsetting.set_brightness_level(brightness[i])
            brightness1 = dbsetting.get_brightness_level()
            self.d.sleep()
            time.sleep(2)
            self.d.wakeup()
            brightness2 = dbsetting.get_brightness_level()
            assert brightness1 == brightness2, "Device display and brightness change after suspend and resume devices."

    def test_Backlight_powerOff(self):
        """
        Display is lightening and the color and brightness is ok.
        """
        screen = g_common_obj.adb_cmd_capture_msg("dumpsys window |grep mScreenOn")
        assert screen.find("mScreenOnFully=true") != -1, "Display is not lightening"
        brightness = dbsetting.get_brightness_level()
        print brightness
        assert brightness >= 0 and brightness <= 255, "brightness is not OK and brightness is %s" % brightness
        g_common_obj.adb_cmd_capture_msg("input keyevent 26")
        time.sleep(5)
        backlight = windows_info.get_dumpsys_backlight()
        assert backlight == 0, "Screen is not off"
        g_common_obj.adb_cmd_capture_msg("input keyevent 26")
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")
        common.unlock_screen()
        g_common_obj.assert_exp_happens()
