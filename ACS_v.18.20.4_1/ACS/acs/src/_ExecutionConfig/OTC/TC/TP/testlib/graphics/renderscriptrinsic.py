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
@summary: Class for RenderScriptIntrinsic_app-debug APK UI Operation
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
            return uiobj.wait("exists", timeout * 1000)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    @property
    def wait_gone(self):
        """ wait until the ui object gone """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("gone", timeout * 1000)
        return _wait


class RenderScriptIntrinsic:

    '''
    classdocs
    '''
    apk_name = "RenderScriptIntrinsic_app-debug.apk"
    pkg_name = "com.example.android.renderscriptintrinsic"
    activity_name = "com.example.android.renderscriptintrinsic.MainActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch RenderScriptIntrinsic via adb am command
        """
        print "Launch RenderScriptIntrinsic by adb am"
        g_common_obj.launch_app_am(\
            RenderScriptIntrinsic.pkg_name, RenderScriptIntrinsic.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop RenderScriptIntrinsic via adb am command
        """
        print "Stop RenderScriptIntrinsic by adb am"
        g_common_obj.stop_app_am(RenderScriptIntrinsic.pkg_name)

    def select_emboss(self):
        """ Chose the emboss option
        """
        self._device(resourceIdMatches=".*radio1").click()
        time.sleep(3)

    def select_hue(self):
        """ Chose the emboss option
        """
        self._device(resourceIdMatches=".*radio2").click()
        time.sleep(3)

    def swipe_the_slider(self):
        """ swipe the slider
        """
        self._device(resourceIdMatches=".*seekBar1").swipe.right(steps=100)
        time.sleep(2)

