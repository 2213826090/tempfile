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
    pkg_name = "com.google.android.videos"
    activity_name = "com.google.android.videos.mobile.presenter.activity.HomeActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def set_environment(self):
        """ init the test environment
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_cast = config.read(cfg_file, 'chromecast_device')
        self.nexus_device = cfg_cast.get("device")
        cmd = "-s %s logcat -d -c" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        print "[clear log]"

    def launch_app_am(self):
        """ Launch videos via adb am command
        """
        print "Launch videos by adb am"
        g_common_obj.launch_app_am(\
            ChromeCastImpl.pkg_name, ChromeCastImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self._device(resourceId="com.google.android.videos:id/end_button").exists:
            self._device(resourceId="com.google.android.videos:id/end_button").click()
        if self._device(resourceId="com.google.android.videos:id/end_button").exists:
            self._device(resourceId="com.google.android.videos:id/end_button").click()
        if self._device(resourceId="com.google.android.videos:id/end_button").exists:
            self._device(resourceId="com.google.android.videos:id/end_button").click()

    @staticmethod
    def stop_app_am():
        """ Stop videos via adb am command
        """
        print "Stop videos by adb am"
        g_common_obj.stop_app_am(ChromeCastImpl.pkg_name)

    def play_video_via_chromecast(self):
        """ play a video via chromecast
        """
        self._device(description="Cast").click()
        time.sleep(2)
        if self._device(className="android.widget.LinearLayout", index="0").exists:
            self._device(className="android.widget.LinearLayout", index="0").click()
        if self._device(text="Disconnect").exists:
            self._device.press.back()
        time.sleep(2)
        self._device(scrollable=True).scroll.to(text="Trailer")
        time.sleep(2)
        self._device(text="Trailer").click()
        time.sleep(2)

    def check_chromecast_status(self):
        """ track aplog to check the chromecast status
        """
        cmd = "-s %s logcat -d > /tmp/chromecast.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/chromecast.log|grep 'cast.receiver.IpcChannel'"\
            ), "The video used chromecast didn't Launch"
        time.sleep(20)

    def check_chromecast_pause(self):
        """ check the chromecast pause
        """
        cmd = "-s %s logcat -d > /tmp/chromecast.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/chromecast.log|grep PAUSED"\
            ), "The video used chromecast didn't pause"
        time.sleep(5)

    def check_chromecast_play(self):
        """ check the chromecast playback
        """
        cmd = "-s %s logcat -d > /tmp/chromecast.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/chromecast.log|grep PAUSED"\
            ), "The video used chromecast didn't play"
        time.sleep(5)

    def pause_chromecast(self):
        """ pause the chromecast video
        """
        self._device.press.back()
        if not self._device(description="Cast").exists:
            self._device(scrollable=True).scroll.vert.toBeginning()
        self._device(description="Cast").click()
        time.sleep(2)
        self._device(resourceId=
            "com.google.android.videos:id/mr_control_play_pause").click()
        time.sleep(2)

    def play_chromecast(self):
        """ play the chromecast video
        """
        self._device(resourceId=
            "com.google.android.videos:id/mr_control_play_pause").click()
        time.sleep(2)

    def stop_casting(self):
        """ Stop Casting
        """
        print "Stop Casting"
        if self._device(text="Stop casting").exists:
            self._device(text="Stop casting").click.wait()
