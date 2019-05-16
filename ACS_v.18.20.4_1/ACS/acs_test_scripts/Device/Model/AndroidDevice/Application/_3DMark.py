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
@summary: This script implements the 3D Mark performance benchmark
@since: 09/07/2014
@author: skoolanx
"""
import re
import time
import sys
import os
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

"""Load PCCG Library"""
library_path = os.path.join(os.path.abspath(os.path.curdir), "_ExecutionConfig/PCCG/library")
sys.path.append(library_path)
from PCCGUtilities import *


class _3DMark(IAndroidPackage):
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
        self._path_ref = os.path.join("BENCHMARKS", "3DMARK")
        self.is_lower_better = False
        self.__pattern = "%s&quot;:&quot;([0-9.]+)"
        self._results = {"gt1_fps": [], "demo_fps": [], "score": [], "physics_score": [], "graphics_score": [],
                         "physics_fps": [], "gt2_fps": []}
        self.__device = device
        self.result_file_name = "/data/data/com.futuremark.dmandroid.application/shared_prefs/activity.ResultsActivity.xml"

    def __get_screenshot(self):
        """
        Capture the screenshot at end of the test
        """
        report_dir = self.__device.get_report_tree()
        self.__3d_mark_report_path = os.path.join(report_dir.get_report_path(), "_3dmark_result")

        filename = os.path.join(self.__3d_mark_report_path, "%s %s.png" % (self._arguments, time.ctime().replace(':', '-')))
        self.__device.screenshot(filename=filename)
        self._logger.info("Screenshot placed at %s" % filename)

    def _fetch_result(self):
        """
        Get score for 3D Mark
        """
        res_act = 'com.futuremark.dmandroid.application/com.futuremark.dmandroid.application.activity.ResultsActivity'
        for _ in range(60):
            time.sleep(1)
            if str(PCCGUtilities.get_current_activity()) == res_act:
                time.sleep(2)
                self._logger.debug("++ Taking screenshots of 3DMark %s results" % self._arguments)
                self.__get_screenshot()
                break

        self._logger.info("++ fetch 3DMark results")

        self._result_file = self.result_file_name
        self._logger.info("++ 3DMark result file path: %s" % self._result_file)
        result = self._fetch_file()

        for key in self._results.keys():
            msg = self.__pattern % key
            match = re.search(msg, result)
            if match:
                self._logger.info("++ score: " + key + " = " + (match.group(0).replace('&quot;', '').split(':')[1]))
                self._results[key].append(float(match.group(0).replace('&quot;', '').split(':')[1]))
            else:
                raise DeviceException("Can not fetch 3DMark result for: " + key)

    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        self._logger.info("++ wait while tests are running")
        score_str = 'adb shell cat %s | grep -o score' % self.result_file_name
        for _ in range(timeout):
            time.sleep(1)
            score_occurred = PCCGUtilities.get_stdout(score_str)
            if isinstance(score_occurred, list):
                if score_occurred[0] == 'score':
                    time.sleep(5)
                    break
        self._logger.info("++ end of wait")

    def go_back_to_menu(self):
        self._logger.info("go back to menu !")

    def drive(self):
        """
        Drive the application
        """
        self._logger.info("++ drive")
        cmd = "rm -rf %s" % self.result_file_name
        self.adb_shell(cmd, 2)

        # To show the list of Icestorm tests
        command = ["DPAD_DOWN", "ENTER"]
        self._keyevent.scenario(command, 2.0)
        time.sleep(1)

        test_to_run = self._arguments
        time.sleep(5)
        if len(test_to_run) == 0:
            msg = "TC ARGUMENTS value is empty.(Possible values are : RUN_UNLIMITED, RUN_EXTREME, RUN_ICESTORM)"
            raise AcsConfigException(msg)

        time.sleep(5)
        if test_to_run == "RUN_EXTREME":
            command = ["DPAD_DOWN", "DPAD_DOWN", "ENTER"]
            self._keyevent.scenario(command, 2.0)

        elif test_to_run == "RUN_UNLIMITED":
            command = ["DPAD_DOWN", "ENTER"]
            self._keyevent.scenario(command, 2.0)

        elif test_to_run == "RUN_ICESTORM":
            command = ["DPAD_DOWN", "DPAD_DOWN", "DPAD_DOWN", "ENTER"]
            self._keyevent.scenario(command, 2.0)
        else:
            msg = "TC ARGUMENTS value is invalid.(Possible values are : RUN_UNLIMITED, RUN_EXTREME, RUN_ICESTORM)"
            raise AcsConfigException(msg)

    def start(self):
        """
        Start application
        """
        self._logger.debug("++ clear logcat buffer")
        PCCGUtilities.clear_logcat()

        self._logger.info("++ start")
        IAndroidPackage.start(self)
        self._logger.info("++ wait few secs till tests are ready to run")
        for _ in range(100):
            time.sleep(1)
            ok_shown = PCCGUtilities.find_in_logcat(pat='onProductModelChange', l_opt='BaseWebViewFragmentActivity:I *:S')
            if isinstance(ok_shown, str):
                self._logger.debug("++ Drop down button displayed")
                time.sleep(5)
                break

    def post_install(self):
        """
        Post-installation actions
        """
        self._logger.debug("++ clear logcat buffer")
        PCCGUtilities.clear_logcat()

        self._logger.info("++ start app")
        IAndroidPackage.start(self)

        self._logger.info("++ wait till [OK, let's go! button] is displayed")
        # 10 minutes max timeout
        for _ in range(600):
            time.sleep(1)
            ok_shown = PCCGUtilities.find_in_logcat(pat='onProductModelChange', l_opt='BaseWebViewFragmentActivity:I *:S')
            if isinstance(ok_shown, str):
                self._logger.info("++ OK button displayed")
                time.sleep(5)
                break
        else:
            raise DeviceException("Could not find OK button on the welcome window")

        self._logger.info("++ press OK button")
        command = ["DPAD_DOWN", "ENTER"]
        self._keyevent.scenario(command, 2.0)

        self._logger.info("++ wait while searching .dlc files")
        # 30 minutes max timeout
        for _ in range(1800):
            time.sleep(1)
            dlc_inst = PCCGUtilities.find_in_logcat(pat='dlcState\\":\\"INSTALLED', l_opt='BaseWebViewFragment:E *:S')
            if isinstance(dlc_inst, str):
                self._logger.info("++ dlc files installed")
                time.sleep(10)
                break
        else:
            raise DeviceException("Application failed to install .dlc files")

        self._logger.info("++ go to Home screen")
        self._keyevent.scenario(["BACK", "BACK", "BACK", "HOME"], 2.0)

        self.clear_cache()

