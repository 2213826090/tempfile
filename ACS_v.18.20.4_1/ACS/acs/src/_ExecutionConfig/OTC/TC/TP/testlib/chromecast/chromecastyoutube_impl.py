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
@since: 02/09/2015
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
    pkg_name = "com.google.android.youtube"
    activity_name = "com.google.android.apps.youtube.app.WatchWhileActivity"

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
        """ Launch youtube via adb am command
        """
        print "Launch youtube by adb am"
        g_common_obj.launch_app_am(\
            ChromeCastImpl.pkg_name, ChromeCastImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop youtube via adb am command
        """
        print "Stop youtube by adb am"
        g_common_obj.stop_app_am(ChromeCastImpl.pkg_name)

    def skip_welcome(self):
        """skip welcome for youtube
        """
        if self._device(text="Skip").exists:
            self._device(text="Skip").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)

    def play_video_via_chromecast(self):
        """ play a video via ChromeCast
        """
        self._device(resourceId=
            "com.google.android.youtube:id/thumbnail").click()
        time.sleep(15)
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        time.sleep(1)
        self._device(resourceId=
            "com.google.android.youtube:id/media_route_button").click()
        time.sleep(5)
        self._device(resourceId="com.google.android.youtube:id/mr_chooser_route_name").click()
        time.sleep(20)

    def check_chromecast_status(self):
        """ track aplog to check the chromecast status
        """
        cmd = "-s %s logcat -d > /tmp/chromecast.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/chromecast.log|grep 'YouTube'"\
            ), "The video used chromecast didn't play"

    def close_video_via_chromecast(self):
        """ close the video played via ChromeCast
        """
        self._device(resourceId=
            "com.google.android.youtube:id/media_route_button").click.wait()
        time.sleep(1)
        self._device(text="Stop casting").click.wait()
        time.sleep(2)
