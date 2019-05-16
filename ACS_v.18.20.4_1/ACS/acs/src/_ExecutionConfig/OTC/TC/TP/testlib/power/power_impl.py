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

import time
import random
import os
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class PowerImpl():
    def __init__(self):
        self._device = g_common_obj.get_device()

    @staticmethod
    def __get_state(state):
        """
        @summary: Check device state
        @parameter: {unknown, device}
        @return: True or False
        """
        pipe = os.popen("adb get-state")
        result = pipe.read().rstrip()
        print "state %s" % result
        if result.find(state) == 0:
            print "Get state %s" % result
            return True
        return False

    def power_off_by_press_powerkey(self):
        """
        @summary: Long press power key to power off DUT
        """
        cmd_press = "adb shell sendevent /dev/input/event0 0001 116 1;\
        adb shell sendevent /dev/input/event0 0000 0000 00000000;\
        adb shell sleep 2;\
        adb shell sendevent /dev/input/event0 0001 116 00000000;\
        adb shell sendevent /dev/input/event0 0000 0000 00000000"

        os.system(cmd_press)
        time.sleep(1)
        assert self._device(resourceId="android:id/message", \
            text="Power off").exists, \
        "[INFO]: The contentPanel does not come out"
        self._device(resourceId="android:id/message", \
            text="Power off").click()
        time.sleep(20)
        assert self.__get_state("unknown"), \
        "[INFO]: Device doesn't power off in 20s"