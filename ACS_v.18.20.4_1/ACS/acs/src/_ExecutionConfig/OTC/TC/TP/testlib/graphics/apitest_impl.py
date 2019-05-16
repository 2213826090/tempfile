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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
# and approved by Intel in writing.
"""
@summary: ChromeExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import logcat


class APITestImpl(object):

    """ com.intel.test.apitests """

    CONFIG_FILE = 'tests.common.apitest.conf'
    PACKAGE_NAME = 'com.intel.test.apitests'

    DisplayMetricsTestDriver_test = 'com.intel.test.apitests.tests.DisplayMetricsTestDriver#testDisplayAllFields'
    DisplayMetricsTestDriver_runner = 'com.intel.test.apitests/com.intel.test.apitests.runners.DisplayMetricsTestRunner'

    def __init__(self):
        super(APITestImpl, self).__init__()
        self.device = g_common_obj.get_device()

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "APITestImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))

    def setup(self):
        apk_path = self.arti.get(self.config.get("apk"))
        cmd = "install -r %s" % (apk_path)
        print g_common_obj.adb_cmd_common(cmd)

    def clean(self):
        cmd = "uninstall %s" % (self.PACKAGE_NAME)
        print g_common_obj.adb_cmd_common(cmd)

    def run_DisplayMetricsTest(self):
        time_mark = logcat.get_device_time_mark()
        cmd = "am instrument -e class %s -w %s"\
            % (self.DisplayMetricsTestDriver_test, self.DisplayMetricsTestDriver_runner)
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        assert 'OK ' in msg,\
            "[FAILURE] Failed am instrument\n%s\n%s" % (cmd, msg)
        logs = logcat.get_device_log(time_mark, "*:D|grep API")
        assert len(logs) > 0,\
            "[FAILURE] Failed Catch logs"
        return logs

    def verify_screen_size(self, msg):
        height_pixels = re.findall(r'.* heightPixels = (\d+).*', msg, re.IGNORECASE)
        width_pixels = re.findall(r'.* widthPixels = (\d+).*', msg, re.IGNORECASE)
        print "[Debug] height_pixels=%s,width_pixels=%s"\
            % (height_pixels, width_pixels)
        assert height_pixels and width_pixels,\
            "[FAILURE] No field values, height_pixels=%s, width_pixels=%s" % (height_pixels, width_pixels)
        assert all([int(each) > 0 for each in height_pixels]),\
            "Invalid height %s" % (height_pixels)
        assert all([int(each) > 0 for each in width_pixels]),\
            "Invalid width %s" % (width_pixels)
