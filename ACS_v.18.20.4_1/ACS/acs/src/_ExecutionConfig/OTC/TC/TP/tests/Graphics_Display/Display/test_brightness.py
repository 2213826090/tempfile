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
@summary: test_Brightness
@since: 07/29/2015
@author: Zhang,RongX Z
"""

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.common import dbsetting
from testlib.graphics.common import windows_info
from testlib.graphics.brightness import BrightnessImpl
import testlib.graphics.common as common

class BrightnessTest(UIATestBase):
    """
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(BrightnessTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.brightness = BrightnessImpl()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(BrightnessTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        dbsetting.set_brightness_level(125)
        self.brightness.change_adaptive_brightness_status("off")
        common.stop_settings_am()

    def test_Display_AdaptiveBrightness(self):
        """
        Verify if adaptive brightness works in dark and bright environment.
        """
        print "[RunTest]: %s" % self.__str__()
        self.brightness.change_adaptive_brightness_status("off")
        dbsetting.set_brightness_level(255)
        backlight1=windows_info.get_dumpsys_backlight()
        self.brightness.change_adaptive_brightness_status("on")
        backlight2=windows_info.get_dumpsys_backlight()
        assert backlight2<backlight1,"[FAILUTE]adaptive brightness doesn't works in dark and bright environment."

    def test_Brightness_SetDisplayBrightness(self):
        """
        Check screen display during boot and change the main display backlight (min value to max value).
        """
        print "[RunTest]: %s" % self.__str__()
        brightness_value = [10,100,150,200,255]
        backlight = []
        for i in range(0,len(brightness_value)):
            dbsetting.set_brightness_level(brightness_value[i])
            backlight_value=windows_info.get_dumpsys_backlight()
            backlight.append(backlight_value)
            if i > 0:
                assert backlight[i] > backlight[i-1],\
                "[FAILURE] backlight doesn't change according to brightness changing.\
                backlight_before is %s,backlight_after is %s" %(backlight[i-1],backlight[i])

    def test_BrightestDisplay_SuspendResume(self):
        """
        Check the stability of DUT display by suspend/resume it in brightest brightness.
        """
        self.brightness.change_adaptive_brightness_status("off")
        for i in range(0,10):
            self.brightness.test_brightness_suspend_resume()

    def test_Backlight_PWROFF(self):
        """
        Check Pressing PWR key to OFF primary display.
        """
        self.brightness.test_Backlight_powerOff()