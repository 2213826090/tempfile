"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL PCCG PSTV-BA
@summary: This script implements the Vellamo benchmark for performance measurement
@since: 12/08/2014
@author: skoolanx
"""
import re
import time
import os
import sys
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase

"""Load PCCG Library"""
library_path = os.path.join(os.path.abspath(os.path.curdir), "_ExecutionConfig/PCCG/library")
sys.path.append(library_path)
from PCCGUtilities import *


class Vellamo(IAndroidPackage, AndroidDeviceBase):
    """
    Implementation of Benchmark_Pi as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        @type device: Device
        @param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self.is_lower_better = False
        self.__count = 0
        self.html5_cords = 0
        self.__device = device
        self.TIMEOUT = 10
        self.__pattern = ".*=\"%s\" value=\"(?P<result>[0-9\.]+)\".*"
        self._results = {"chapter-html5-score": []}
        self.result_file_name = "/data/data/com.quicinc.vellamo/shared_prefs/v2sp.xml"
        self._path_ref = os.path.join("BENCHMARKS", "VELLAMO")


    def __get_screenshot(self):
        """
        Capture the screenshot at end of the test
        """
        report_dir = self.__device.get_report_tree()
        self.__vellamo_report_path = os.path.join(report_dir.get_report_path(), "vellamo_result")

        filename = os.path.join(self.__vellamo_report_path, "Vellamo %s.png" % time.ctime().replace(':', '-'))
        self.__device.screenshot(filename=filename)
        self._logger.info("Screenshot placed at %s" % filename)

    def _fetch_result(self):
        """
        Get score for Vellamo
        """
        self._logger.info("++ fetch results")
        self._result_file = self.result_file_name
        result = self._fetch_file()

        for key in self._results.keys():
            msg = self.__pattern % key
            match = re.search(msg, result)
            if match:
                self._logger.info("++ score: " + key + " = " + match.group("result"))
                self._results[key].append(int(match.group("result")))
            else:
                raise DeviceException("Can not fetch Vellamo result for: " + key)

        self._logger.debug("++ Taking screenshot of Vellamo result")
        self.__get_screenshot()

        if PCCGUtilities.find_view(attr="text",
                                   value="Your device.s score and system information will be sent to the server.",
                                   clear_dump=False):
            PCCGUtilities.tap(attr="text", value="No", get_dump=False)
        time.sleep(2)

        if PCCGUtilities.find_view(attr="text", value="Vellamo EULA", clear_dump=False):
            PCCGUtilities.tap(attr="text", value="Accept", get_dump=False)
            self._logger.debug("Accepted Vellamo EULA")
        time.sleep(2)

    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        self._logger.info("++ wait while tests are running")
        score_str = 'adb shell cat %s | grep -o chapter-html5-score' % self.result_file_name
        for _ in range(timeout):
            time.sleep(1)
            score_occurred = PCCGUtilities.get_stdout(score_str)
            if isinstance(score_occurred, list):
                if score_occurred[0] == 'chapter-html5-score':
                    time.sleep(5)
                    break
        self._logger.info("++ end of wait")

    def drive(self):
        """
        Drive the application
        """
        self._logger.info("++ drive")
        self.adb_shell("rm -rf /data/data/com.quicinc.vellamo/shared_prefs/v2sp.xml", self.TIMEOUT)
        self.__count += 1

        if self.__count == 1:
            self.html5_cords = PCCGUtilities.find_view(multi_attr_value='index="1".*class="android.view.View".*')
            if not self.html5_cords:
                raise DeviceException("Could not find HTML5 button to get co-ordinates.")

        if not PCCGUtilities.find_view(attr="resource-id", value="com.quicinc.vellamo:id/frag_launcher_buttons"):
            raise DeviceException("Could not find HTML5 button to run benchmark.")

        self.adb_shell("input tap %d %d" % (self.html5_cords[0], self.html5_cords[1]), self.TIMEOUT)
        time.sleep(2)

        # To proceed without network connection
        if PCCGUtilities.find_view(attr="text", value="? No Network Connection", clear_dump=False):
            PCCGUtilities.tap(attr="text", value="Yes", get_dump=False)
            self._logger.debug("Agreed to proceed without network connection")

        # To skip the tutorial
        if PCCGUtilities.find_view(attr="text",
                                   value="Since this is your first time running the benchmarks, "
                                         "would you like a tutorial as we progress?", clear_dump=False):
            PCCGUtilities.tap(attr="text", value="No", get_dump=False)
            self._logger.debug("Skipped tutorial")

    def start(self):
        """
        Start application
        """
        self._logger.info("++ start")
        IAndroidPackage.start(self)
        time.sleep(2)
        if PCCGUtilities.find_view(attr="text", value="Vellamo EULA", clear_dump=False):
            PCCGUtilities.tap(attr="text", value="Accept", get_dump=False)
            self._logger.debug("Accepted Vellamo EULA")

    def post_install(self):
        """
        Post-installation actions
        """
        self.disable_jni()

        self.adb_shell("reboot", self.TIMEOUT)
        self.__device.disconnect_board()
        self.__device.connect_board()

        self._logger.info("wait 30 secs after reboot")
        time.sleep(30)

        self.clear_cache()
