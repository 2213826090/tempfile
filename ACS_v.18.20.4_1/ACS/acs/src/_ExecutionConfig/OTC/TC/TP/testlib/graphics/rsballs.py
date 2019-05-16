#Copyright (C) 2014  Jin, YingjunX <yingjunx.jin@intel.com>
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

'''
@summary: Class for RsBalls APK UI Operation
@since: 01/14/2015
@author: Yingjun Jin
'''

import os
import time
from testlib.util.common import g_common_obj

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=10):
            return uiobj.wait("exists", timeout*1000)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    @property
    def wait_gone(self):
        """ wait until the ui object gone """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("gone", timeout*1000)
        return _wait

class RsBalls:
    '''
    classdocs
    '''
    apk_name = "RsBalls.apk"
    pkg_name = "com.example.android.rs.balls"
    activity_name = "com.example.android.rs.balls.Balls"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch RsBalls via adb am command
        """
        print "Launch RsBalls by adb am"
        g_common_obj.launch_app_am(\
            RsBalls.pkg_name, RsBalls.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        assert self._device(text="RsBalls").exists,"[FAILURE] Launch RsBall.apk failed."

    @staticmethod
    def stop_app_am():
        """ Stop RsBalls via adb am command
        """
        print "Stop RsBalls by adb am"
        g_common_obj.stop_app_am(RsBalls.pkg_name)

    def rotate_screen(self):
        '''
        Rotate screen.
        '''
        print "[Info] ---Rotate screen"
        self._device.orientation = "l"
        time.sleep(1)
        self._device.orientation = "r"
        time.sleep(1)
        self._device.orientation = "n"
        time.sleep(2)
        assert self._device(text="RsBalls").exists,"[FAILURE]After rotating screen,RsBall.apk quit."
