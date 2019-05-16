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
@since: 02/11/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle

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
    pkg_name = "com.netflix.mediaclient"
    activity_name = "com.netflix.mediaclient.ui.signup.SignupActivity"

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
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, 'netflix')
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)
        self.id = config_handle.read_configuration('google_account', 'user_name', '/etc/oat/', 'sys.conf')
        self.pwd = config_handle.read_configuration('google_account', 'password', '/etc/oat/', 'sys.conf')

    def launch_app_am(self):
        """ Launch netflix via adb am command
        """
        print "Launch netflix by adb am"
        g_common_obj.launch_app_am(\
            ChromeCastImpl.pkg_name, ChromeCastImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop netflix via adb am command
        """
        print "Stop netflix by adb am"
        g_common_obj.stop_app_am(ChromeCastImpl.pkg_name)

    def login_account(self):
        """ login google account to play video
        """
        print "[login account]"
        self._device(text="Sign In").click()
        time.sleep(2)
        os.system("adb shell input text %s" % self.id)
        time.sleep(2)
        self._device(
            resourceId="com.netflix.mediaclient:id/login_password").click()
        time.sleep(2)
        os.system("adb shell input text %s" % self.pwd)
        time.sleep(2)
        self._device(
            resourceId="com.netflix.mediaclient:id/login_action_btn"
            ).click()
        time.sleep(30)
        if self._device(
            resourceId="com.netflix.mediaclient:id/loading_view").exists:
            time.sleep(30)
        if self._device(description="test").exists:
            self._device(description="test").click()
        time.sleep(10)
        if self._device(text="No Thanks").exists:
            self._device(text="No Thanks").click()

    def play_video_via_chromecast(self):
        """ play a video via ChromeCast
        """
        self._device(resourceId="com.netflix.mediaclient:id/cw_view_img"
            ).click()
        time.sleep(15)
        self._device(resourceId=
            "com.netflix.mediaclient:id/surface").click()
        time.sleep(2)
        self._device(description="Play On").click()
        time.sleep(3)
        self._device(className="android.widget.RelativeLayout", index="1").click()
        time.sleep(60)

    def check_chromecast_status(self):
        """ track aplog to check the chromecast status
        """
        cmd = "-s %s logcat -d > /tmp/chromecast.log" % self.nexus_device
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system(
            "cat /tmp/chromecast.log|grep 'Netflix'"\
            ), "The video used chromecast didn't play"

    def set_default_screen(self):
        self._device.orientation = "n"

    @staticmethod
    def uninstall_app():
        """ uninstall the netflix
        """
        print "Uninstall the netflix "
        cmd = 'uninstall %s' % ChromeCastImpl.pkg_name
        g_common_obj.adb_cmd_common(cmd)
