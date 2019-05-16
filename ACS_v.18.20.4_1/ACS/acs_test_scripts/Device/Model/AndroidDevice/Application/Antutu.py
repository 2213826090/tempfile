"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This script implements the Antutu benchmark for performance measurement
:since: 16/04/2013
:author: pbluniex
"""
import re
import os
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class Antutu(IAndroidPackage):

    """
    Implementation of Antutu as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self.is_lower_better = False
        self.__attempt = 1
        self._path_ref = os.path.join("BENCHMARKS", "ANTUTU")
        self.__data_path = "/data/data/com.antutu.ABenchMark"
        self.__pattern = "AnTuTuBenchmarkScore:\s*%s\s*:\s*"
        self._results = {"memory": [], "integer": [], "float": [], "2d": [], "3d": [],
                         "database": [], "sdwrite": [], "sdread": [], "score": []}
        self._result_v4 = {"memory": [], "integer": [], "float": [], "2d": [], "3d": [],
                             "database": [], "score": [], "multitask": [], "ram": [],
                             "dalvik": [], "renderscript": [], "storage": []}
        self._result_v5 = {"memory": [], "integer": [], "float": [], "2d": [], "3d": [],
                         "database": [], "multitask": [], "score": [], "integersingle": [], "floatsingle": [],
                        "dalvik": [], "renderscript": [], "storage": [], "ram": []}

        self.__available_tests = ["CPU_MEMORY", "2D", "3D",
                                  "DATABASE_IO", "SDCARD_IO",
                                  None]
        self.__run_num = 0

    def _fetch_result(self):
        """
        Return the score of benchmark run
        """
        for key in self._results.keys():
            msg = self.__pattern % key
            res = self._get_device_logger().get_message_triggered_status("regex:" + msg)

            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't get score for %s" % key)

            matchobj = re.search("%s(?P<result>\d+)" % msg, res[0])
            result = matchobj.group("result")
            if result is not None and result.isdigit():
                self._results[key].append(int(matchobj.group("result")))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't fetch result for %s" % key)

            self._get_device_logger().remove_trigger_message(msg)

    def wait(self, timeout):
        """
        Wait until the end of the run

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        if not self._get_device_logger().\
           is_message_received("regex:" + self.__pattern % "score", timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout while executing AnTuTu")

    def __select_tests(self):
        """
        Select the test defined in ARGUMENTS parameter
        """
        command = ["dpad_right", "dpad_down"]

        for test in self.__available_tests:
            # Deselection of test
            if test != self._arguments and self.__run_num == 0:
                command.append("enter")

            if test is not None:
                command.append("tab")

        command.extend(["dpad_left", "enter"])
        self._keyevent.scenario(command)
        self.__run_num += 1

    def __select_tests_v4(self):
        """
        select test for the V4 version of antutu
        """
        #from home to test page
        command = ["tab", "dpad_down", "enter"]
        self._keyevent.scenario(command, 2.0)

        time.sleep(2)

        #start the tests
        command = ["tab" for _ in range(0, 5)]
        command.append("enter")
        self._keyevent.scenario(command, 2.0)

    def __select_tests_v5(self):
        """
        select test for the V5 version of antutu
        """
        if self.__attempt == 1:
            #get build type
            cmd = "getprop ro.product.cpu.abi"
            output = self.adb_shell(cmd, 10)

            #if buils is a 64b type, need to escape from a popup
            if output.find("64") != -1:
                self._logger.info("Device is running on 64b build")
                #start the test
                command = ["escape", "dpad_up", "dpad_up", "dpad_up", "dpad_down", "enter"]
                self._keyevent.scenario(command, 2.0)
            else:
                #start the test
                command = ["dpad_up", "dpad_down", "enter"]
                self._keyevent.scenario(command, 2.0)
        else:
            #restart the test
            command = ["dpad_up", "dpad_down", "enter", "enter"]
            self._keyevent.scenario(command, 2.0)

    def start(self):
        """
        Start application
        """
        # Clean cache before starting
        cmd = "find %s -maxdepth 1 -mindepth 1"\
              "|grep -v lib|grep -v shared_prefs"\
              "|xargs rm -r && echo 0" % self.__data_path
        self.adb_shell(cmd, 10)
        version = self._get_version()
        if "4.0.3" in version:
            self._results = self._result_v4
        elif "5.6" in version:
            self._results = self._result_v5

        IAndroidPackage.start(self)

    def drive(self):
        """
        Drive the application from home to test runner
        """
        for key in self._results.keys():
            pattern = self.__pattern % key
            self._get_device_logger().add_trigger_message("regex:" + pattern)

        time.sleep(10.0)
        version = self._get_version()
        if "4.0.3" in version:
            self.__select_tests_v4()
        elif "5.6" in version:
            self.__select_tests_v5()
        else:
            self.__select_tests()

    def go_back_home(self):
        """
        Go back to the application menu
        """
        for key in self._results.keys():
            pattern = self.__pattern % key
            self._get_device_logger().add_trigger_message("regex:" + pattern)
        self.__attempt += 1
        version = self._get_version()
        if "4.0.3" in version:
            time.sleep(1)
            cmd = ["back"]
            self._keyevent.scenario(cmd, 2.0)
            time.sleep(1)
        elif "5.6" in version:
            time.sleep(1)
            cmd = ["back", "dpad_up", "dpad_left"]
            self._keyevent.scenario(cmd, 2.0)
            time.sleep(1)
        else:
            pass

    def post_install(self):
        """
        Post-installation actions
        """
        preferences = "<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n"\
                      "<map>\n"\
                      "<int name='lastVersionCode' value='3311' />\n"\
                      "</map>"

        cmd = "mkdir %s/shared_prefs" % self.__data_path
        cmd += " && echo \"%s\" > %s/shared_prefs/com.antutu.ABenchMark_preferences.xml" % \
               (preferences, self.__data_path)

        self.adb_shell(cmd, 10)
