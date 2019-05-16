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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for Memtrack operation
@since: 02/05/2015
@author: Yingjun Jin
'''

import re
import time
from testlib.util.common import g_common_obj
from testlib.util.common import Common


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


class MemTrackImpl:

    '''
    classdocs
    '''
    apk_name = "GLBenchmark_2.5.1.apk"
    pkg_name = "com.glbenchmark.glbenchmark25"
    activity_name = "com.glbenchmark.activities.MainActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch GLBenchmark_2.5.1 via adb am command
        """
        print "Launch GLBenchmark_2.5.1 by adb am"
        g_common_obj.launch_app_am(\
            MemTrackImpl.pkg_name, MemTrackImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop GLBenchmark_2.5.1 via adb am command
        """
        print "Stop GLBenchmark_2.5.1 by adb am"
        g_common_obj.stop_app_am(MemTrackImpl.pkg_name)

    @staticmethod
    def uninstall_app():
        """ uninstall the GLBenchmark_2.5.1
        """
        print "Uninstall the GLBenchmark_2.5.1 "
        cmd = 'uninstall com.glbenchmark.glbenchmark25'
        g_common_obj.adb_cmd_common(cmd)

    def check_graphics_mem(self):
        """ check the graphics mem
        """
        output = g_common_obj.adb_cmd_capture_msg(\
            "dumpsys meminfo | grep -E 'Graphics|EGL mtrack' | awk '{print $1}'")
        print output
        size = re.sub(r'\D', '', output)
        assert int(size) > 0, "The Graphics mem is 0"
        time.sleep(2)

        return int(size)

    def run_performance_test(self):
        """ run the performance tests
        """
        self._device(resourceId="com.glbenchmark.glbenchmark25:id/text").click()
        time.sleep(2)
        self._device(resourceId="com.glbenchmark.glbenchmark25:id/linearLayout1").click()
        time.sleep(2)
        self._device(resourceId="com.glbenchmark.glbenchmark25:id/buttonStart").click()
        time.sleep(4)

    def compare_graphics_mem(self):
        """ compare the mem before and after run the benchmark
        """
        self.result_after = g_common_obj.adb_cmd_capture_msg(\
            "dumpsys meminfo | grep -E 'Graphics|EGL mtrack' | awk '{print $1}'")
        print self.result_after
        differ = abs(int(self.result_after) - int(self.result_before))
        print differ
        assert differ > 0, "The Graphics mem is not changed"
        time.sleep(2)

    def set_default_screen(self):
        self._device.orientation = "n"
