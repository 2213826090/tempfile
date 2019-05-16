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
:summary: This script implements the quadrant benchmark for performance measurement
:since: 16/04/2013
:author: pbluniex
"""
import os
import re
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class Quadrant(IAndroidPackage):
    """
    Implementation of caffeinemark as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "QUADRANT")
        self.is_lower_better = False

        self.__pattern = ".* %s aggregate score is (?P<result>\d+)"
        self._result = {}

        self.__aggregate_score = {"cpu": [], "memory": [], "io": [],
                                  "g2d": [], "g3d": [], "score": []}

        self.__subscore_pattern = ".* %s executed .*, reference .*, score: (?P<score>\d+)"
        self.__subscores = {"cpu_branching_logic": [],
                            "cpu_matrix_int": [],
                            "cpu_matrix_long": [],
                            "cpu_matrix_short": [],
                            "cpu_matrix_byte": [],
                            "cpu_matrix_float": [],
                            "cpu_matrix_double": [],
                            "cpu_checksum": [],
                            "io_fs_write": [],
                            "io_fs_read": [],
                            "io_db_write": [],
                            "io_db_read": [],
                            "g2d_fractal": [],
                            "g3d_corridor": [],
                            "g3d_planet": [],
                            "g3d_dna": []}

    def __get_pattern(self, pattern, key):
        """
        Return the pattern to search
        """
        if key == "score":
            _pattern = "benchmark"
        else:
            _pattern = "benchmark_%s" % key

        return pattern % _pattern

    def __fetch_benchmark_result(self):
        """
        Fetch agregate results
        """
        for key in self.__aggregate_score.keys():
            search = self.__get_pattern(self.__pattern, key)
            res = self._get_device_logger().get_message_triggered_status("regex:" + search)
            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't get result for %s" % key)

            matchobj = re.match(search, res[0])
            result = matchobj.group("result")
            if result is not None and result.isdigit():
                self.__aggregate_score[key].append(int(matchobj.group("result")))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Can't fetch result")

            self._get_device_logger().remove_trigger_message("regex:" + search)

    def __fetch_cpu_result(self):
        """
        Fetch CPU subscore results
        """
        for key in self.__subscores.keys():
            search = self.__get_pattern(self.__subscore_pattern, key)
            res = self._get_device_logger().get_message_triggered_status("regex:" + search)
            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't get score for %s" % key)

            matchobj = re.match(search, res[0])
            score = matchobj.group("score")
            if score is not None and score.isdigit():
                self.__subscores[key].append(int(matchobj.group("score")))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't fetch score for %s" % key)

            self._get_device_logger().remove_trigger_message("regex:" + search)

    def _fetch_result(self):
        """
        Fetch the score of quadrant run
        """
        self.__fetch_benchmark_result()
        self.__fetch_cpu_result()

        for key, value in self.__aggregate_score.items():
            self._results[key] = value

        for key, value in self.__subscores.items():
            self._results[key] = value

    def wait(self, timeout):
        """
        Wait until the end of the run

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        if not self._get_device_logger().\
           is_message_received("regex:" + self.__pattern % "benchmark", timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Execution test timeout has been reached")

    def drive(self):
        """
        Drive the application to run the benchmark
        """
        for key in self.__aggregate_score.keys():
            msg = "regex:" + self.__get_pattern(self.__pattern, key)
            self._get_device_logger().add_trigger_message(msg)

        for key in self.__subscores.keys():
            msg = "regex:" + self.__get_pattern(self.__subscore_pattern, key)
            self._get_device_logger().add_trigger_message(msg)

        cmd = ["move_home", "enter", "enter"]
        self._keyevent.scenario(cmd, 1)

    def go_back_home(self):
        """
        Go back to the home screen
        """
        time.sleep(1)
        cmd = ["back"]
        self._keyevent.scenario(cmd, 1)
        time.sleep(1)

    def post_install(self):
        """
        Pre-installation actions
        """
        self.disable_jni()

        # Prevent licence screen to be printed
        directory = "/data/data/com.aurorasoftworks.quadrant.ui.professional/shared_prefs"
        preffile = directory + "/QuadrantProfessionalActivity.xml"
        cmd = "mkdir %s;"\
              "echo \"<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n"\
              "    <map>\n"\
              "        <boolean name='greeting_shown' value='true' />\n"\
              "    </map>\" "\
              " > %s" % (directory, preffile)

        self.adb_shell(cmd, 10)

        # Force GPU rendering to true
        cmd = "setprop persist.sys.ui.hw 1"
        self.adb_shell(cmd, 10)

    def uninstall(self):
        IAndroidPackage.uninstall(self)

        # Force GPU rendering to false
        cmd = "setprop persist.sys.ui.hw 0"
        self.adb_shell(cmd, 10)
