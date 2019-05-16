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
@summary: This file implements for battery
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj

class BatteryImpl:
    """
    Battery Test Impl Class
    """
    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    @staticmethod
    def get_battery_level():
        """
            get battery level
        """
        print "[INFO] get battery level"
        cmd = "dumpsys battery|grep level"
        level = g_common_obj.adb_cmd_capture_msg(cmd).split()
        assert not level[1] is None, "Get level error!"
        print "[INFO]: Current battery level is %d" % int(level[1])
        return int(level[1])

    def check_battery_more(self, battery_level):
        """
        @summary: check current battery more than batter_level
        """
        level = self.get_battery_level()
        error_msg = "[FAILURE] Check battery failed %d < %s" % \
        (level, battery_level)
        assert int(level) >= int(battery_level), error_msg

    def check_battery_less(self, battery_level):
        """
        @summary: check current battery less than batter_level
        """
        level = self.get_battery_level()
        error_msg = "[FAILURE] Check battery failed %d > %s" % \
        (level, battery_level)
        assert int(level) <= int(battery_level), error_msg