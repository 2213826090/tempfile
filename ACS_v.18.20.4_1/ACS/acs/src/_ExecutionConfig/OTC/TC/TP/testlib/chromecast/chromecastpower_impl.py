#Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for ChromeCast operation
@since: 02/11/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory


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
            return uiobj.wait("exists", timeout*500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

class ChromeCastImpl:
    '''
    classdocs
    '''

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def connect_chromecast(self):
        """ connect to the chromecast
        """
        print "Connecting to chromecast"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Cast").click()
        time.sleep(10)
        if self._device(text="No nearby devices were found.").exists:
            time.sleep(30)
        assert self._device(className="android.widget.RelativeLayout"
            ).exists ,"No nearby chromecast adapter"
        self._device(className="android.widget.RelativeLayout"
            ).click()
        time.sleep(5)
        self.stop_settings_am()

    def disconnect_chromecast(self):
        """ disconnect the chromecast
        """
        print "Disconnecting chromecast"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Cast").click()
        time.sleep(5)
        if self._device(text="Connected"):
            self._device(className="android.widget.RelativeLayout"
            ).click()
            time.sleep(2)
            self._device(text="DISCONNECT").click()
            time.sleep(2)
        self.stop_settings_am()

    def set_sleep_time_1min(self):
        """ set sleep time 1min
        """
        print "Set auto sleep time for 1 minute"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Sleep").click()
        time.sleep(2)
        self._device(text="1 minute").click()
        time.sleep(2)
        self.stop_settings_am()

    def set_sleep_time_10min(self):
        """ set sleep time 1min
        """
        print "Set auto sleep time for 1 minute"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Sleep").click()
        time.sleep(2)
        self._device(text="10 minutes").click()
        time.sleep(2)
        self.stop_settings_am()

    def checkout_stay_awake(self):
        """ checkout the stay awake when charge
        """
        print "Checkout the stay awake options"
        self.launch_settings_am()
        self._device(scrollable=True).scroll.to(text="Developer options")
        time.sleep(2)
        self._device(text="Developer options").click()
        time.sleep(2)
        if self._device(text="Stay awake").right(
            resourceId="android:id/checkbox") != None:
            if self._device(text="Stay awake").right(
                resourceId="android:id/checkbox").checked:
                self._device(text="Stay awake").right(
                    resourceId="android:id/checkbox").click()
        if self._device(className="android.widget.LinearLayout", index=2
            ).child(text="ON").exists:
            self._device(className="android.widget.LinearLayout", index=2
                ).child(text="ON").click()
        time.sleep(2)
        self.stop_settings_am()

    def check_stay_awake(self):
        """ check the stay awake when charge
        """
        print "Check the stay awake options"
        self.launch_settings_am()
        self._device(scrollable=True).scroll.to(text="Developer options")
        time.sleep(2)
        self._device(text="Developer options").click()
        time.sleep(2)
        if self._device(text="Stay awake"
            ).right(resourceId="android:id/checkbox") != None:
            if not self._device(text="Stay awake"
                ).right(resourceId="android:id/checkbox").checked:
                self._device(text="Stay awake"
                    ).right(resourceId="android:id/checkbox").click()
        if self._device(className="android.widget.LinearLayout", index=2
            ).child(text="OFF").exists:
            self._device(className="android.widget.LinearLayout", index=2
                ).child(text="OFF").click()
        time.sleep(2)
        self.stop_settings_am()

    @staticmethod
    def wait_dut_sleep_1min():
        """ wait 1mins that DUT sleep
        """
        print "idle for going to sleep!"
        time.sleep(70)

    @staticmethod
    def sleep_time(sleeptime):
        """ wait DUT to sleep
        """
        print "idle for going to sleep!"
        time.sleep(int(sleeptime))

    def set_sleep_time_30min(self):
        """ set sleep time back to 30mins
        """
        print "Set sleep time for 30mins"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Sleep").click()
        time.sleep(2)
        self._device(text="30 minutes").click()
        time.sleep(2)
        self.stop_settings_am()

    def suspend_and_resume(self):
        """ suspend and resume the DUT
        """
        self._device.press.power()
        time.sleep(5)
        self._device.press.power()
        time.sleep(2)

    def check_screen_status(self):
        screen = g_common_obj.adb_cmd_capture_msg("dumpsys window |grep mScreenOn")
        print screen
        assert screen.find("mScreenOnFully=true") ==-1,"Display is lightening"
