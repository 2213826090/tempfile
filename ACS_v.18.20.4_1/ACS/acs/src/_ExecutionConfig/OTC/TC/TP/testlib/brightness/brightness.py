#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: This file implements for brightness
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj


class BrightnessBSPImpl:
    """
    Battery Test Impl Class
    """
    rootFolder = "/sys/class/backlight/"
    _old_brightness = "/sys/class/backlight/brightness"
    _old_actualbrightness = "/sys/class/backlight/actual_brightness"
    pFolders = g_common_obj.adb_cmd_capture_msg("find /sys/class/backlight/").splitlines()
    pFolder = [i for i in pFolders if "intel_backlight" in i]
    Brightness = [pFolder[0] + "/brightness" if len(pFolder) == 1 else _old_brightness][0]
    Actual_Brightness = [pFolder[0] + "/actual_brightness" if len(pFolder) == 1 else _old_actualbrightness][0]

    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        check_cmd = "cat " + BrightnessBSPImpl.Brightness
        self.level = int(g_common_obj.adb_cmd_capture_msg(check_cmd))

    @staticmethod
    def get_brightness_level():
        """
            get brightness level
        """
        print "[INFO] get brightness level"
        cmd = "cat " + BrightnessBSPImpl.Actual_Brightness
        level = g_common_obj.adb_cmd_capture_msg(cmd)
        assert not level is None, "Get brightness error!"
        print "[IO]: Current brightness level is %d" % (255 - int(level))
        return (255 - int(level))

    def set_brightness_level(self, level):
        """
            set brightness level
        """
        print "[INFO] set brightness level"
        if int(level) > 255:
            print "[WARNING] The brightness should be in [0, 255]!"
            print "[INFO] Set brightness to 255!"
            level = 255
        elif int(level) < 0:
            print "[WARNING] The brightness should be in [0, 255]!"
            print "[INFO] Set brightness to 0!"
            level = 0
        else:
            level = int(level)
            print "[INFO] Set brightness to %d!" % level
        check_cmd = "cat " + BrightnessBSPImpl.Brightness
        cmd = "\"echo " + str(level) + " > " + BrightnessBSPImpl.Brightness + "\""
        g_common_obj.adb_cmd_capture_msg(cmd)
        time.sleep(10)
        e_level = g_common_obj.adb_cmd_capture_msg(check_cmd)
        assert not int(level) == e_level, "Set brightness error!"

    def reset(self):
        """
        reset brightness to Orignal level
        """
        print "[INFO] Orignal brightness is %d" % self.level
        self.set_brightness_level(self.level)